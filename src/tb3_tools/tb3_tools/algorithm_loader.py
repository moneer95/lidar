"""Filesystem-based algorithm discovery for tb3_nav."""

from __future__ import annotations

import importlib.util
import inspect
from pathlib import Path
from typing import Dict, Type

from tb3_tools.algorithm_api import NavigationAlgorithm


def discover_algorithms(algorithms_dir: Path) -> Dict[str, Type[NavigationAlgorithm]]:
    """Discover algorithms from .py files in the given directory."""
    registry: Dict[str, Type[NavigationAlgorithm]] = {}

    if not algorithms_dir.exists() or not algorithms_dir.is_dir():
        return registry

    for file_path in sorted(algorithms_dir.glob("*.py")):
        if file_path.name.startswith("_"):
            continue

        module_name = f"tb3_ext_alg_{file_path.stem}"
        spec = importlib.util.spec_from_file_location(module_name, file_path)
        if spec is None or spec.loader is None:
            continue
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        for _, cls in inspect.getmembers(module, inspect.isclass):
            if not issubclass(cls, NavigationAlgorithm) or cls is NavigationAlgorithm:
                continue
            algo_name = getattr(cls, "name", "").strip()
            if not algo_name:
                continue
            if algo_name in registry:
                raise ValueError(f'Duplicate algorithm name "{algo_name}" detected.')
            registry[algo_name] = cls

    return registry

