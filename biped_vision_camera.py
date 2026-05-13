from __future__ import annotations

import ipaddress
import json
import socket
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
from urllib.parse import urlparse

import cv2
import numpy as np
import requests


REQUEST_TIMEOUT = 0.35
DEFAULT_IP_WEBCAM_PORT = 8080
CACHE_FILE = Path(__file__).resolve().parent / "biped_vision_camera_cache.json"


@dataclass
class CameraSource:
    source_id: str
    name: str
    kind: str
    value: int | str
    base_url: Optional[str] = None
    preview_url: Optional[str] = None


def _read_cache() -> dict:
    try:
        return json.loads(CACHE_FILE.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _write_cache(payload: dict) -> None:
    CACHE_FILE.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def get_cached_ip_webcam_url() -> Optional[str]:
    payload = _read_cache()
    cached_url = payload.get("last_ip_webcam_url")
    if isinstance(cached_url, str) and cached_url:
        return cached_url
    return None


def cache_ip_webcam_url(url: str) -> None:
    payload = _read_cache()
    payload["last_ip_webcam_url"] = url
    _write_cache(payload)


def _local_ipv4_networks() -> List[ipaddress.IPv4Network]:
    networks: list[ipaddress.IPv4Network] = []
    seen: set[str] = set()
    hostname = socket.gethostname()
    addresses: set[str] = set()

    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            addresses.add(info[4][0])
    except socket.gaierror:
        pass

    try:
        addresses.add(socket.gethostbyname(hostname))
    except socket.gaierror:
        pass

    for address in addresses:
        if address.startswith("127."):
            continue
        parts = address.split(".")
        if len(parts) != 4:
            continue
        network_str = ".".join(parts[:3]) + ".0/24"
        if network_str in seen:
            continue
        seen.add(network_str)
        networks.append(ipaddress.ip_network(network_str, strict=False))

    return networks


def _local_ipv4_addresses() -> List[str]:
    addresses: list[str] = []
    seen: set[str] = set()
    hostname = socket.gethostname()

    try:
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            address = info[4][0]
            if address.startswith("127.") or address in seen:
                continue
            seen.add(address)
            addresses.append(address)
    except socket.gaierror:
        pass

    try:
        address = socket.gethostbyname(hostname)
        if not address.startswith("127.") and address not in seen:
            addresses.append(address)
    except socket.gaierror:
        pass

    return addresses


def _is_ip_webcam_host(ip: str, port: int = DEFAULT_IP_WEBCAM_PORT) -> Optional[CameraSource]:
    base_url = f"http://{ip}:{port}"
    checks = [
        f"{base_url}/status.json",
        f"{base_url}/shot.jpg",
        base_url,
    ]

    for url in checks:
        try:
            response = requests.get(url, timeout=REQUEST_TIMEOUT)
        except requests.RequestException:
            continue

        if response.status_code != 200:
            continue

        content_type = response.headers.get("content-type", "").lower()
        body = response.text[:300].lower() if "text" in content_type else ""

        if "ip webcam" in body or "json" in content_type or "image/jpeg" in content_type:
            shot_url = f"{base_url}/shot.jpg"
            return CameraSource(
                source_id=f"ip-{ip}",
                name=f"IP Webcam ({ip})",
                kind="ip_webcam",
                value=shot_url,
                base_url=base_url,
                preview_url=shot_url,
            )

    return None


def _build_ip_candidates(networks: List[ipaddress.IPv4Network], mode: str) -> List[str]:
    cached_url = get_cached_ip_webcam_url()
    cached_ip = None
    if cached_url:
        try:
            cached_ip = urlparse(cached_url).hostname
        except ValueError:
            cached_ip = None

    candidates: list[str] = []
    seen: set[str] = set()
    local_addresses = _local_ipv4_addresses()

    def add_candidate(ip: str) -> None:
        if ip in seen:
            return
        seen.add(ip)
        candidates.append(ip)

    if cached_ip:
        add_candidate(cached_ip)

    for address in local_addresses:
        octets = address.split(".")
        if len(octets) != 4:
            continue
        prefix = ".".join(octets[:3])
        host_number = int(octets[3])
        if mode == "quick":
            likely_hosts = [host_number + offset for offset in range(-8, 9)]
            likely_hosts += list(range(100, 111))
            likely_hosts += list(range(120, 141))
            likely_hosts += list(range(150, 171))
            likely_hosts += list(range(180, 201))
            likely_hosts += list(range(2, 20))
            for host in likely_hosts:
                if 1 <= host <= 254:
                    add_candidate(f"{prefix}.{host}")

    for network in networks:
        hosts = [str(host) for host in network.hosts()]
        if mode == "quick":
            for ip in hosts[-40:] + hosts[99:140] + hosts[:20]:
                add_candidate(ip)
        else:
            for ip in hosts[:254]:
                add_candidate(ip)

    return candidates


def discover_ip_webrtcams(mode: str = "quick") -> List[CameraSource]:
    return discover_ip_webcams(mode=mode)


def discover_ip_webcams(mode: str = "quick") -> List[CameraSource]:
    networks = _local_ipv4_networks()
    if not networks:
        return []

    candidates = _build_ip_candidates(networks, mode=mode)
    results: list[CameraSource] = []
    with ThreadPoolExecutor(max_workers=24) as executor:
        futures = {executor.submit(_is_ip_webcam_host, ip): ip for ip in candidates}
        for future in as_completed(futures):
            source = future.result()
            if source:
                cache_ip_webcam_url(str(source.value))
                results.append(source)

    results.sort(key=lambda item: item.name)
    return results


def fetch_ip_webcam_frame(url: str, timeout: float = 3.0) -> np.ndarray:
    response = requests.get(url, timeout=timeout)
    response.raise_for_status()
    image = cv2.imdecode(np.frombuffer(response.content, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Failed to decode camera frame.")
    cache_ip_webcam_url(url)
    return image
