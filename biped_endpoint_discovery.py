from __future__ import annotations

import ipaddress
import json
import socket
import urllib.error
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_BIPED_MDNS = "dume-biped.local"
IDENTIFY_PATH = "/api/identify"
DISCOVERY_TIMEOUT = 1.0
DISCOVERY_WORKERS = 32


@dataclass(frozen=True)
class ResolvedBipedEndpoint:
    base_url: str
    source: str
    identify_payload: dict | None = None


NO_PROXY_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def normalize_base_url(candidate: str | None) -> str | None:
    if not candidate:
        return None
    normalized = candidate.strip()
    if not normalized:
        return None
    if "://" not in normalized:
        normalized = f"http://{normalized}"
    return normalized.rstrip("/")


def canonical_device_base_url(candidate: str, payload: dict | None = None) -> str:
    normalized = normalize_base_url(candidate) or ""
    if not payload:
        return normalized
    ip_address = str(payload.get("ip_address", "")).strip()
    if ip_address:
        return normalize_base_url(ip_address) or normalized
    return normalized


def extract_host(candidate: str | None) -> str | None:
    normalized = normalize_base_url(candidate)
    if not normalized:
        return None
    return urllib.parse.urlparse(normalized).hostname


def load_device_cache(repo_root: Path) -> dict:
    cache_path = repo_root / "device_cache.json"
    if not cache_path.exists():
        return {}
    try:
        return json.loads(cache_path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addresses.add(ip)
    except Exception:
        pass
    return sorted(addresses)


def probable_subnets(repo_root: Path, extra_hosts: Iterable[str] | None = None) -> list[ipaddress.IPv4Network]:
    candidates = list(local_ipv4_addresses())
    cache = load_device_cache(repo_root)
    if cache.get("last_ip"):
        candidates.insert(0, str(cache["last_ip"]))
    for host in extra_hosts or []:
        if host and host not in candidates:
            candidates.insert(0, host)

    seen: set[str] = set()
    subnets: list[ipaddress.IPv4Network] = []
    for ip in candidates:
        try:
            network = ipaddress.ip_network(f"{ip}/24", strict=False)
        except ValueError:
            continue
        if network.network_address.is_loopback:
            continue
        key = str(network)
        if key in seen:
            continue
        seen.add(key)
        subnets.append(network)
    return subnets


def probe_biped_endpoint(base_url: str, timeout: float = DISCOVERY_TIMEOUT) -> ResolvedBipedEndpoint | None:
    normalized = normalize_base_url(base_url)
    if not normalized:
        return None
    url = f"{normalized}{IDENTIFY_PATH}"
    request = urllib.request.Request(url, headers={"Connection": "close", "User-Agent": "Biped-Discovery/1.0"}, method="GET")
    try:
        with NO_PROXY_OPENER.open(request, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception:
        return None
    if not payload.get("ok", True):
        return None
    if payload.get("device_type") not in {"robot_arm", "robot_biped"}:
        return None
    return ResolvedBipedEndpoint(
        base_url=canonical_device_base_url(normalized, payload),
        source="probe",
        identify_payload=payload,
    )


def discover_biped_endpoint(config_base_url: str | None, repo_root: Path, *, allow_subnet_scan: bool = True) -> ResolvedBipedEndpoint | None:
    cache = load_device_cache(repo_root)
    candidate_urls: list[tuple[str, str]] = []
    for value, source in [
        (config_base_url, "config"),
        (cache.get("last_success_url"), "device_cache"),
        (f"http://{cache['last_mdns_hostname']}" if cache.get("last_mdns_hostname") else None, "device_cache_mdns"),
        (f"http://{cache['last_hostname']}.local" if cache.get("last_hostname") else None, "device_cache_hostname"),
        (f"http://{DEFAULT_BIPED_MDNS}", "default_mdns"),
    ]:
        normalized = normalize_base_url(value)
        if normalized and all(existing[0] != normalized for existing in candidate_urls):
            candidate_urls.append((normalized, source))

    for candidate, source in candidate_urls:
        resolved = probe_biped_endpoint(candidate, timeout=1.0)
        if resolved is not None:
            return ResolvedBipedEndpoint(base_url=resolved.base_url, source=source, identify_payload=resolved.identify_payload)

    if not allow_subnet_scan:
        return None

    extra_hosts = [extract_host(item[0]) for item in candidate_urls]
    hosts: list[str] = []
    for network in probable_subnets(repo_root, [host for host in extra_hosts if host]):
        hosts.extend(str(host) for host in network.hosts())

    with ThreadPoolExecutor(max_workers=DISCOVERY_WORKERS) as executor:
        futures = {
            executor.submit(probe_biped_endpoint, f"http://{host}", DISCOVERY_TIMEOUT): host
            for host in hosts
        }
        for future in as_completed(futures):
            resolved = future.result()
            if resolved is not None:
                return ResolvedBipedEndpoint(base_url=resolved.base_url, source="subnet_scan", identify_payload=resolved.identify_payload)
    return None


# Backward-compatible aliases for older imports.
ResolvedDumeEndpoint = ResolvedBipedEndpoint
probe_dume_device = probe_biped_endpoint
discover_dume_endpoint = discover_biped_endpoint

