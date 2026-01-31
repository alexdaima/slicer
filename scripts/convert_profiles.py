#!/usr/bin/env python3
"""
BambuStudio Profile Converter

This script converts BambuStudio printer and filament profiles from their
inheritance-based format to a flattened format suitable for the Rust slicer.

For printers:
- One file per printer+nozzle combo is converted to one file per printer model
- Nozzle configurations are nested under 'nozzle_configs'

For filaments:
- Resolves inheritance chains
- Extracts all relevant properties

Usage:
    python convert_profiles.py [--profiles-dir <path>] [--output-dir <path>]

Example:
    python convert_profiles.py --profiles-dir ../BambuStudio/resources/profiles --output-dir ../data
"""

import argparse
import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Optional

# ============================================================================
# Utilities
# ============================================================================


def load_json(path: Path) -> dict:
    """Load a JSON file, handling BambuStudio's format quirks."""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: dict):
    """Save JSON with pretty formatting."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  Saved: {path.name}")


def unwrap_array_value(value: Any) -> Any:
    """
    Unwrap single-element arrays to their contained value.
    BambuStudio often wraps values in arrays like ["0.4"] instead of 0.4
    """
    if isinstance(value, list) and len(value) == 1:
        return value[0]
    return value


def parse_numeric(value: Any) -> Optional[float]:
    """Try to parse a value as a number."""
    if value is None or value == "nil":
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def parse_percent(value: Any) -> Optional[float]:
    """Parse a percentage value (e.g., '95%' -> 95.0)."""
    if value is None or value == "nil":
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        value = value.strip()
        if value.endswith("%"):
            try:
                return float(value[:-1])
            except ValueError:
                return None
        try:
            return float(value)
        except ValueError:
            return None
    return None


def slugify(name: str) -> str:
    """Convert a name to a slug ID."""
    slug = name.lower()
    slug = re.sub(r"[^a-z0-9]+", "-", slug)
    slug = slug.strip("-")
    return slug


def get_brand_slug(brand_name: str) -> str:
    """Convert brand name to slug."""
    brand_map = {
        "BBL": "bambu-lab",
        "Bambu Lab": "bambu-lab",
        "Creality": "creality",
        "Prusa": "prusa",
        "Voron": "voron",
        "Anker": "anker",
        "Anycubic": "anycubic",
        "Elegoo": "elegoo",
        "Geeetech": "geeetech",
        "Qidi": "qidi",
        "Tronxy": "tronxy",
        "Vivedino": "vivedino",
        "Voxelab": "voxelab",
    }
    return brand_map.get(brand_name, slugify(brand_name))


# ============================================================================
# Brand Configuration
# ============================================================================

BRAND_CONFIGS = {
    "BBL": {
        "name": "Bambu Lab",
        "slug": "bambu-lab",
        "structure": "corexy",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "input_shaping": True,
            "arc_support": True,
            "firmware_retraction": False,
            "multi_material": True,
            "filament_runout_sensor": True,
            "power_loss_recovery": True,
        },
        "bed": {
            "heated": True,
            "surface_types": [
                "smooth_pei",
                "textured_pei",
                "engineering_plate",
                "high_temp_plate",
            ],
            "default_surface": "textured_pei",
        },
    },
    "Creality": {
        "name": "Creality",
        "slug": "creality",
        "structure": "cartesian",
        "extruder_type": "bowden",
        "features": {
            "auto_bed_leveling": False,
            "pressure_advance": False,
            "input_shaping": False,
            "arc_support": True,
            "firmware_retraction": False,
            "multi_material": False,
            "filament_runout_sensor": False,
            "power_loss_recovery": False,
        },
        "bed": {
            "heated": True,
            "surface_types": ["glass", "spring_steel"],
            "default_surface": "glass",
        },
    },
    "Prusa": {
        "name": "Prusa",
        "slug": "prusa",
        "structure": "cartesian",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "input_shaping": True,
            "arc_support": True,
            "firmware_retraction": True,
            "multi_material": True,
            "filament_runout_sensor": True,
            "power_loss_recovery": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["smooth_pei", "textured_pei", "satin_pei"],
            "default_surface": "textured_pei",
        },
    },
    "Voron": {
        "name": "Voron",
        "slug": "voron",
        "structure": "corexy",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "input_shaping": True,
            "arc_support": True,
            "firmware_retraction": False,
            "multi_material": False,
            "filament_runout_sensor": True,
            "power_loss_recovery": False,
        },
        "bed": {
            "heated": True,
            "surface_types": ["smooth_pei", "textured_pei"],
            "default_surface": "textured_pei",
        },
    },
    "Anker": {
        "name": "Anker",
        "slug": "anker",
        "structure": "cartesian",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "input_shaping": False,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["spring_steel"],
            "default_surface": "spring_steel",
        },
    },
    "Anycubic": {
        "name": "Anycubic",
        "slug": "anycubic",
        "structure": "cartesian",
        "extruder_type": "bowden",
        "features": {
            "auto_bed_leveling": False,
            "pressure_advance": False,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["glass", "ultrabase"],
            "default_surface": "glass",
        },
    },
    "Elegoo": {
        "name": "Elegoo",
        "slug": "elegoo",
        "structure": "cartesian",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["spring_steel"],
            "default_surface": "spring_steel",
        },
    },
    "Geeetech": {
        "name": "Geeetech",
        "slug": "geeetech",
        "structure": "cartesian",
        "extruder_type": "bowden",
        "features": {
            "auto_bed_leveling": False,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["glass"],
            "default_surface": "glass",
        },
    },
    "Qidi": {
        "name": "Qidi",
        "slug": "qidi",
        "structure": "corexy",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "pressure_advance": True,
            "input_shaping": True,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["spring_steel", "textured_pei"],
            "default_surface": "spring_steel",
        },
    },
    "Tronxy": {
        "name": "Tronxy",
        "slug": "tronxy",
        "structure": "cartesian",
        "extruder_type": "bowden",
        "features": {
            "auto_bed_leveling": False,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["glass"],
            "default_surface": "glass",
        },
    },
    "Vivedino": {
        "name": "Vivedino",
        "slug": "vivedino",
        "structure": "corexy",
        "extruder_type": "direct_drive",
        "features": {
            "auto_bed_leveling": True,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["spring_steel"],
            "default_surface": "spring_steel",
        },
    },
    "Voxelab": {
        "name": "Voxelab",
        "slug": "voxelab",
        "structure": "cartesian",
        "extruder_type": "bowden",
        "features": {
            "auto_bed_leveling": False,
            "arc_support": True,
        },
        "bed": {
            "heated": True,
            "surface_types": ["glass"],
            "default_surface": "glass",
        },
    },
}


# ============================================================================
# Printer Profile Converter
# ============================================================================


class PrinterConverter:
    """Converts BambuStudio machine profiles to our format."""

    def __init__(self, brand_dir: Path, brand_id: str):
        self.brand_dir = brand_dir
        self.brand_id = brand_id
        self.brand_config = BRAND_CONFIGS.get(
            brand_id,
            {
                "name": brand_id,
                "slug": slugify(brand_id),
                "structure": "cartesian",
                "extruder_type": "bowden",
                "features": {},
                "bed": {"heated": True, "surface_types": [], "default_surface": ""},
            },
        )
        self.profiles: dict[str, dict] = {}
        self.printer_models: dict[str, dict] = {}
        self.nozzle_profiles: dict[str, dict[str, dict]] = defaultdict(dict)

    def load_all_profiles(self):
        """Load all machine profiles."""
        machine_path = self.brand_dir / "machine"
        if not machine_path.exists():
            print(f"  Warning: No machine directory found at {machine_path}")
            return

        for json_file in sorted(machine_path.glob("*.json")):
            try:
                profile = load_json(json_file)
                name = profile.get("name", json_file.stem)
                self.profiles[name] = profile
            except Exception as e:
                print(f"  Warning: Failed to load {json_file}: {e}")

    def resolve_inheritance(self, profile: dict, visited: Optional[set] = None) -> dict:
        """Resolve profile inheritance, merging parent values."""
        if visited is None:
            visited = set()

        name = profile.get("name", "unknown")
        if name in visited:
            return profile

        visited.add(name)

        inherits = profile.get("inherits")
        if not inherits or inherits not in self.profiles:
            return profile.copy()

        parent = self.resolve_inheritance(self.profiles[inherits], visited)
        resolved = parent.copy()
        resolved.update(profile)

        return resolved

    def identify_printer_models(self):
        """Identify unique printer models."""
        for name, profile in self.profiles.items():
            profile_type = profile.get("type", "")
            if profile_type == "machine_model":
                self.printer_models[name] = profile
                print(f"    Found: {name}")

    def categorize_nozzle_profiles(self):
        """Group nozzle-specific profiles by printer model."""
        for name, profile in self.profiles.items():
            profile_type = profile.get("type", "")

            if profile_type != "machine":
                continue
            if profile.get("instantiation") != "true":
                continue

            printer_model = profile.get("printer_model")
            if not printer_model:
                continue

            nozzle_diameter = profile.get("nozzle_diameter")
            if isinstance(nozzle_diameter, list) and len(nozzle_diameter) > 0:
                nozzle_diameter = nozzle_diameter[0]

            if not nozzle_diameter:
                continue

            resolved = self.resolve_inheritance(profile)
            self.nozzle_profiles[printer_model][str(nozzle_diameter)] = resolved

    def extract_build_volume(self, profile: dict, model_name: str) -> dict:
        """Extract build volume from profile."""
        printable_area = profile.get("printable_area", [])
        x, y, z = 200, 200, 200  # defaults

        if len(printable_area) >= 3:
            try:
                corner = printable_area[2]
                parts = corner.split("x")
                if len(parts) == 2:
                    x = int(parts[0])
                    y = int(parts[1])
            except (ValueError, IndexError, AttributeError):
                pass

        # Try to get Z from profile
        z_val = profile.get("printable_height")
        if z_val:
            parsed_z = parse_numeric(unwrap_array_value(z_val))
            if parsed_z:
                z = int(parsed_z)

        return {"x": x, "y": y, "z": z, "origin": "corner"}

    def extract_limits(self, profile: dict) -> dict:
        """Extract machine movement limits."""

        def get_first(key: str, default: float) -> float:
            val = profile.get(key, [default])
            if isinstance(val, list) and len(val) > 0:
                return parse_numeric(val[0]) or default
            return parse_numeric(val) or default

        return {
            "max_speed": {
                "x": get_first("machine_max_speed_x", 500),
                "y": get_first("machine_max_speed_y", 500),
                "z": get_first("machine_max_speed_z", 20),
                "e": get_first("machine_max_speed_e", 30),
                "travel": get_first("machine_max_speed_x", 500),
                "print": 300,
            },
            "max_acceleration": {
                "x": get_first("machine_max_acceleration_x", 10000),
                "y": get_first("machine_max_acceleration_y", 10000),
                "z": get_first("machine_max_acceleration_z", 500),
                "e": get_first("machine_max_acceleration_e", 5000),
                "extruding": get_first("machine_max_acceleration_extruding", 10000),
                "retracting": get_first("machine_max_acceleration_retracting", 5000),
                "travel": get_first("machine_max_acceleration_travel", 9000),
            },
            "max_jerk": {
                "x": get_first("machine_max_jerk_x", 9),
                "y": get_first("machine_max_jerk_y", 9),
                "z": get_first("machine_max_jerk_z", 3),
                "e": get_first("machine_max_jerk_e", 2.5),
            },
        }

    def extract_retraction(self, profile: dict) -> dict:
        """Extract retraction settings."""

        def get_first(key: str, default: float) -> float:
            val = profile.get(key, [default])
            if isinstance(val, list) and len(val) > 0:
                return parse_numeric(val[0]) or default
            return parse_numeric(val) or default

        return {
            "length": get_first("retraction_length", 0.8),
            "speed": get_first("retraction_speed", 30),
            "deretraction_speed": get_first("deretraction_speed", 30),
            "z_hop": get_first("z_hop", 0.4),
            "minimum_travel": get_first("retraction_minimum_travel", 1.0),
            "wipe": get_first("wipe", 1) == 1,
            "wipe_distance": get_first("wipe_distance", 2.0),
        }

    def extract_nozzle_config(self, profile: dict, nozzle_diameter: str) -> dict:
        """Extract nozzle-specific configuration."""

        def get_first(key: str, default: Any) -> Any:
            val = profile.get(key, default)
            if isinstance(val, list) and len(val) > 0:
                return val[0]
            return val

        nozzle_type = get_first("nozzle_type", "hardened_steel")
        if nozzle_type == "nil" or not nozzle_type:
            nozzle_type = "hardened_steel"

        min_layer = parse_numeric(get_first("min_layer_height", 0.08)) or 0.08
        max_layer = parse_numeric(get_first("max_layer_height", 0.28)) or 0.28

        diameter = float(nozzle_diameter)
        if diameter <= 0.25:
            min_layer = min_layer or 0.04
            max_layer = max_layer or 0.14
        elif diameter <= 0.45:
            min_layer = min_layer or 0.08
            max_layer = max_layer or 0.28
        elif diameter <= 0.65:
            min_layer = min_layer or 0.12
            max_layer = max_layer or 0.42
        else:
            min_layer = min_layer or 0.16
            max_layer = max_layer or 0.56

        config = {
            "diameter": diameter,
            "nozzle_type": nozzle_type,
            "min_layer_height": min_layer,
            "max_layer_height": max_layer,
            "retraction": self.extract_retraction(profile),
        }

        start_gcode = profile.get("machine_start_gcode", "")
        if start_gcode:
            config["start_gcode"] = start_gcode

        return config

    def get_printer_features(self, model_name: str, profile: dict) -> dict:
        """Determine printer features based on model name and profile."""
        features = self.brand_config.get("features", {}).copy()
        name_lower = model_name.lower()

        # Brand-specific feature detection
        if self.brand_id == "BBL":
            features["camera"] = any(
                x in name_lower for x in ["x1", "p1s", "p1p", "a1", "h2", "p2s"]
            )
            features["lidar"] = "x1" in name_lower and "carbon" in name_lower
        elif self.brand_id == "Creality":
            if any(x in name_lower for x in ["k1", "ender-3 v3"]):
                features["auto_bed_leveling"] = True
                features["pressure_advance"] = True
                features["input_shaping"] = True
                features["filament_runout_sensor"] = True
            if "k1" in name_lower:
                features["camera"] = True
        elif self.brand_id == "Prusa":
            if "mk4" in name_lower or "xl" in name_lower:
                features["input_shaping"] = True

        return features

    def get_printer_enclosure(self, model_name: str) -> dict:
        """Determine enclosure settings."""
        name_lower = model_name.lower()

        is_enclosed = False
        has_filtration = False

        if self.brand_id == "BBL":
            is_enclosed = any(x in name_lower for x in ["x1", "p1s", "h2"])
            has_filtration = "x1" in name_lower or "h2" in name_lower
        elif self.brand_id == "Creality":
            is_enclosed = any(x in name_lower for x in ["k1", "ender-6"])
        elif self.brand_id == "Voron":
            is_enclosed = True
        elif self.brand_id == "Qidi":
            is_enclosed = any(x in name_lower for x in ["x-plus", "x-max", "x-cf"])

        return {
            "enclosed": is_enclosed,
            "heated_chamber": False,
            "max_chamber_temperature": 60 if is_enclosed else 40,
            "air_filtration": has_filtration,
        }

    def get_bed_config(self, model_name: str, profile: dict) -> dict:
        """Get bed configuration."""
        bed_config = self.brand_config.get("bed", {}).copy()

        # Try to get max bed temp from profile
        max_temp = profile.get("bed_temperature_range_high")
        if max_temp:
            parsed = parse_numeric(unwrap_array_value(max_temp))
            if parsed:
                bed_config["max_temperature"] = parsed
        else:
            bed_config["max_temperature"] = 110 if self.brand_id == "BBL" else 100

        return bed_config

    def convert_printer_model(self, model_name: str) -> Optional[dict]:
        """Convert a printer model to our format."""
        if model_name not in self.nozzle_profiles:
            print(f"    Warning: No nozzle profiles for {model_name}")
            return None

        nozzle_configs = self.nozzle_profiles[model_name]
        if not nozzle_configs:
            return None

        reference_nozzle = (
            "0.4" if "0.4" in nozzle_configs else list(nozzle_configs.keys())[0]
        )
        reference_profile = nozzle_configs[reference_nozzle]

        model_info = self.printer_models.get(model_name, {})

        printer_id = slugify(model_name)
        brand_name = self.brand_config.get("name", self.brand_id)
        brand_slug = self.brand_config.get("slug", slugify(self.brand_id))

        # Clean up model name (remove brand prefix)
        clean_model = model_name
        for prefix in [brand_name + " ", self.brand_id + " "]:
            if clean_model.startswith(prefix):
                clean_model = clean_model[len(prefix) :]

        converted = {
            "$schema": "../schemas/printer.schema.json",
            "id": printer_id,
            "brand": brand_slug,
            "model": clean_model,
            "description": f"{model_name} - {brand_name} FFF 3D Printer",
            "source": {
                "origin": "BambuStudio",
                "version": "auto-converted",
                "url": f"https://github.com/bambulab/BambuStudio",
            },
            "technology": "FFF",
            "structure": self.brand_config.get("structure", "cartesian"),
            "build_volume": self.extract_build_volume(reference_profile, model_name),
            "default_nozzle_diameter": float(reference_nozzle),
            "extruder": {
                "type": self.brand_config.get("extruder_type", "direct_drive"),
                "filament_diameter": 1.75,
                "count": 1,
            },
            "bed": self.get_bed_config(model_name, reference_profile),
            "enclosure": self.get_printer_enclosure(model_name),
            "limits": self.extract_limits(reference_profile),
            "features": self.get_printer_features(model_name, reference_profile),
            "cooling": {
                "fan_count": 2 if self.brand_id == "BBL" else 1,
                "max_fan_speed": 100,
                "auxiliary_fan": self.brand_id == "BBL",
            },
            "gcode": {
                "flavor": "klipper" if self.brand_id in ["BBL", "Voron"] else "marlin",
                "end_gcode": reference_profile.get("machine_end_gcode", ""),
                "toolchange_gcode": reference_profile.get("change_filament_gcode", ""),
            },
            "nozzle_configs": {},
        }

        for nozzle_diameter, profile in sorted(nozzle_configs.items()):
            converted["nozzle_configs"][nozzle_diameter] = self.extract_nozzle_config(
                profile, nozzle_diameter
            )

        converted["metadata"] = {
            "author": "BambuStudio Auto-Converter",
            "license": "Apache-2.0",
            "tags": self._get_tags(model_name),
        }

        return converted

    def _get_tags(self, model_name: str) -> list[str]:
        """Generate tags for a printer."""
        tags = [self.brand_config.get("slug", slugify(self.brand_id))]
        name_lower = model_name.lower()

        structure = self.brand_config.get("structure", "cartesian")
        if structure:
            tags.append(structure)

        # Speed tags
        if self.brand_id == "BBL":
            tags.append("high-speed")
        elif self.brand_id == "Creality" and "k1" in name_lower:
            tags.append("high-speed")

        # Enclosure tags
        if self.get_printer_enclosure(model_name).get("enclosed"):
            tags.append("enclosed")

        return tags

    def convert_all(self, output_dir: Path) -> int:
        """Convert all printer models and save."""
        output_path = output_dir / "printers"
        output_path.mkdir(parents=True, exist_ok=True)

        converted_count = 0
        for model_name in sorted(self.printer_models.keys()):
            print(f"    Converting: {model_name}")
            converted = self.convert_printer_model(model_name)

            if converted:
                output_file = output_path / f"{converted['id']}.json"
                save_json(output_file, converted)
                converted_count += 1
                nozzles = list(converted["nozzle_configs"].keys())
                print(f"      Nozzles: {', '.join(nozzles)}")

        return converted_count


# ============================================================================
# Filament Profile Converter
# ============================================================================


class FilamentConverter:
    """Converts BambuStudio filament profiles to our format."""

    def __init__(self, profiles_dir: Path):
        self.profiles_dir = profiles_dir
        # Store profiles per-brand to avoid cross-brand inheritance issues
        self.brand_profiles: dict[str, dict[str, dict]] = defaultdict(dict)

    def load_all_profiles(self):
        """Load all filament profiles from all brands."""
        for brand_dir in sorted(self.profiles_dir.iterdir()):
            if not brand_dir.is_dir():
                continue
            if brand_dir.name in [
                "check_duplicated_setting_id.py",
                "check_unused_setting_id.py",
            ]:
                continue

            filament_dir = brand_dir / "filament"
            if not filament_dir.exists():
                continue

            print(f"  Loading filaments from {brand_dir.name}...")
            for json_file in sorted(filament_dir.glob("*.json")):
                try:
                    profile = load_json(json_file)
                    name = profile.get("name", json_file.stem)
                    self.brand_profiles[brand_dir.name][name] = profile
                except Exception as e:
                    print(f"    Warning: Failed to load {json_file.name}: {e}")

            # Also load subdirectories (e.g., BBL/filament/Polymaker/)
            for subdir in filament_dir.iterdir():
                if subdir.is_dir():
                    for json_file in sorted(subdir.glob("*.json")):
                        try:
                            profile = load_json(json_file)
                            name = profile.get("name", json_file.stem)
                            self.brand_profiles[brand_dir.name][name] = profile
                        except Exception as e:
                            print(f"    Warning: Failed to load {json_file.name}: {e}")

    def resolve_inheritance(
        self, profile: dict, brand_id: str, visited: Optional[set] = None
    ) -> dict:
        """Resolve filament profile inheritance within a brand's profiles.

        This ensures we don't accidentally inherit from a different brand's
        fdm_filament_common which might have different defaults (e.g., Voxelab
        uses 2.85mm filament while BBL uses 1.75mm).
        """
        if visited is None:
            visited = set()

        name = profile.get("name", "unknown")
        if name in visited:
            return profile

        visited.add(name)

        inherits = profile.get("inherits")
        if not inherits:
            return profile.copy()

        # Look for parent in the same brand first
        brand_profiles = self.brand_profiles.get(brand_id, {})
        parent_profile = brand_profiles.get(inherits)

        if not parent_profile:
            # Not found - return current profile without further resolution
            return profile.copy()

        parent = self.resolve_inheritance(parent_profile, brand_id, visited)
        resolved = parent.copy()
        resolved.update(profile)

        return resolved

    def identify_instantiable_filaments(self) -> list[tuple[str, str, dict]]:
        """Find all filament profiles that can be used as base definitions.

        We specifically want @base profiles because they contain the canonical
        material definitions. The instantiation=false flag just means the slicer
        shouldn't show them directly to users (they're meant to be inherited from),
        but they ARE the proper source of filament data.
        """
        filaments = []
        seen_base_names = set()

        # First pass: collect all @base profiles (these are the canonical sources)
        for brand_id, profiles in self.brand_profiles.items():
            for name, profile in profiles.items():
                # Skip internal template profiles
                if name.startswith("fdm_filament_"):
                    continue
                if name.endswith("_common"):
                    continue
                if "filament_id_map" in name or "filament_name_map" in name:
                    continue
                if "filaments_color_codes" in name:
                    continue

                # Must be a filament type profile
                profile_type = profile.get("type", "")
                if profile_type and profile_type != "filament":
                    continue

                # We want @base profiles as the canonical source
                # These are the base definitions that printer-specific profiles inherit from
                # Note: @base profiles have instantiation=false but that's fine -
                # they're still the proper source of filament data
                # Only collect from BBL brand for @base profiles to get consistent defaults
                if "@base" in name and brand_id == "BBL":
                    # Extract base name without @base suffix
                    base_name = name.replace(" @base", "").replace("@base", "")
                    if base_name not in seen_base_names:
                        filaments.append((brand_id, name, profile))
                        seen_base_names.add(base_name)

        # Second pass: include profiles that don't have @base versions
        # These are standalone filament definitions (common in non-BBL brands)
        for brand_id, profiles in self.brand_profiles.items():
            for name, profile in profiles.items():
                if name.startswith("fdm_filament_"):
                    continue
                if name.endswith("_common"):
                    continue

                profile_type = profile.get("type", "")
                if profile_type and profile_type != "filament":
                    continue

                # Skip printer-specific variants (contain @ but not @base)
                if "@" in name and "@base" not in name:
                    continue

                # Skip @base (already handled above)
                if "@base" in name:
                    continue

                # Check if we already have a @base version for this filament
                base_name = name
                if base_name not in seen_base_names:
                    # Only include if it looks like a material profile
                    if any(
                        prefix in name
                        for prefix in [
                            "Generic ",
                            "Bambu ",
                            "PolyLite ",
                            "PolyTerra ",
                            "Overture ",
                            "eSUN ",
                            "SUNLU ",
                        ]
                    ):
                        # Skip if marked non-instantiable AND there's likely a @base version
                        # (for non-@base profiles, instantiation=false means skip)
                        if profile.get("instantiation") == "false":
                            continue
                        filaments.append((brand_id, name, profile))
                        seen_base_names.add(base_name)

        return filaments

    def extract_filament_type(self, name: str, profile: dict) -> str:
        """Determine the material type."""
        filament_type = unwrap_array_value(profile.get("filament_type", "PLA"))
        if filament_type and filament_type != "nil":
            return filament_type

        # Try to infer from name
        name_upper = name.upper()
        for mat in [
            "PLA",
            "PETG",
            "ABS",
            "ASA",
            "TPU",
            "PA",
            "PC",
            "PVA",
            "HIPS",
            "PP",
            "PPS",
            "PPA",
        ]:
            if mat in name_upper:
                return mat

        return "PLA"

    def extract_temperatures(self, profile: dict) -> dict:
        """Extract temperature settings."""

        def get_val(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_numeric(unwrap_array_value(val))
            return parsed if parsed is not None else default

        return {
            "nozzle": {
                "min": get_val("nozzle_temperature_range_low", 190),
                "max": get_val("nozzle_temperature_range_high", 240),
                "default": get_val("nozzle_temperature", 220),
                "first_layer": get_val("nozzle_temperature_initial_layer", 220),
            },
            "bed": {
                "default": get_val("hot_plate_temp", 60),
                "first_layer": get_val("hot_plate_temp_initial_layer", 60),
                "smooth_pei": get_val("hot_plate_temp", 60),
                "textured_pei": get_val("textured_plate_temp", 60),
                "engineering_plate": get_val("eng_plate_temp", 60),
                "cool_plate": get_val("cool_plate_temp", 35),
            },
            "chamber": {
                "recommended": get_val("chamber_temperatures", 0),
                "required": False,
            },
        }

    def extract_flow(self, profile: dict) -> dict:
        """Extract flow settings."""

        def get_val(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_numeric(unwrap_array_value(val))
            return parsed if parsed is not None else default

        return {
            "ratio": get_val("filament_flow_ratio", 1.0),
            "max_volumetric_speed": get_val("filament_max_volumetric_speed", 12.0),
        }

    def extract_cooling(self, profile: dict) -> dict:
        """Extract cooling settings."""

        def get_val(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_numeric(unwrap_array_value(val))
            return parsed if parsed is not None else default

        def get_percent(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_percent(unwrap_array_value(val))
            return parsed if parsed is not None else default

        return {
            "fan_min_speed": get_val("fan_min_speed", 35),
            "fan_max_speed": get_val("fan_max_speed", 100),
            "fan_below_layer_time": get_val("fan_cooling_layer_time", 60),
            "disable_fan_first_layers": int(get_val("close_fan_the_first_x_layers", 1)),
            "full_fan_speed_layer": int(get_val("full_fan_speed_layer", 0)),
            "slow_down_layer_time": get_val("slow_down_layer_time", 8),
            "slow_down_min_speed": get_val("slow_down_min_speed", 10),
            "overhang_fan_speed": get_val("overhang_fan_speed", 100),
            "overhang_fan_threshold": get_percent("overhang_fan_threshold", 50),
        }

    def extract_retraction(self, profile: dict) -> dict:
        """Extract retraction settings."""

        def get_val(key: str, default: float) -> Optional[float]:
            val = profile.get(key)
            if (
                val is None
                or val == "nil"
                or (isinstance(val, list) and len(val) > 0 and val[0] == "nil")
            ):
                return default
            parsed = parse_numeric(unwrap_array_value(val))
            return parsed if parsed is not None else default

        return {
            "length": get_val("filament_retraction_length", None),
            "speed": get_val("filament_retraction_speed", None),
            "deretraction_speed": get_val("filament_deretraction_speed", None),
            "minimum_travel": get_val("filament_retraction_minimum_travel", None),
            "z_hop": get_val("filament_z_hop", None),
            "wipe": get_val("filament_wipe", None) == 1
            if get_val("filament_wipe", None) is not None
            else None,
            "wipe_distance": get_val("filament_wipe_distance", None),
        }

    def extract_physical_properties(self, profile: dict, material_type: str) -> dict:
        """Extract physical properties."""

        def get_val(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_numeric(unwrap_array_value(val))
            return parsed if parsed is not None else default

        def get_percent_as_ratio(key: str, default: float) -> float:
            val = profile.get(key)
            if val is None:
                return default
            parsed = parse_percent(unwrap_array_value(val))
            return parsed if parsed is not None else default

        # Material-specific defaults
        density_defaults = {
            "PLA": 1.24,
            "PETG": 1.27,
            "ABS": 1.04,
            "ASA": 1.07,
            "TPU": 1.21,
            "PA": 1.14,
            "PC": 1.20,
            "PVA": 1.23,
        }

        return {
            "diameter": get_val("filament_diameter", 1.75),
            "density": get_val(
                "filament_density", density_defaults.get(material_type, 1.24)
            ),
            "spool_weight": 1000,
            "glass_transition_temperature": get_val("temperature_vitrification", 60),
            "vitrification_temperature": get_val("temperature_vitrification", 100),
            "shrinkage": get_percent_as_ratio("filament_shrink", 100),
            "hygroscopic": material_type in ["PA", "PVA", "PC", "TPU"],
            "abrasive": "-CF" in material_type.upper()
            or "-GF" in material_type.upper(),
            "flexible": material_type in ["TPU", "TPE"],
        }

    def extract_special_properties(
        self, name: str, material_type: str, profile: dict
    ) -> dict:
        """Extract special properties from name and profile."""
        name_lower = name.lower()
        mat_upper = material_type.upper()

        return {
            "translucent": "translucent" in name_lower,
            "glow_in_dark": "glow" in name_lower,
            "metallic": "metal" in name_lower,
            "silk": "silk" in name_lower,
            "matte": "matte" in name_lower,
            "wood_filled": "wood" in name_lower,
            "carbon_fiber": "-cf" in name_lower or "carbon" in name_lower,
            "glass_fiber": "-gf" in name_lower,
            "high_speed": "hf" in name_lower
            or "high speed" in name_lower
            or "high flow" in name_lower,
            "requires_enclosure": mat_upper in ["ABS", "ASA", "PC", "PA", "PPS", "PPA"],
            "requires_hardened_nozzle": "-cf" in name_lower or "-gf" in name_lower,
            "requires_drying": mat_upper in ["PA", "PVA", "PC", "TPU", "PETG"],
            "drying_temperature": 50 if mat_upper in ["PLA", "PETG", "TPU"] else 80,
            "drying_time": 4 if mat_upper in ["PLA", "PETG"] else 8,
        }

    def get_filament_brand(self, name: str) -> str:
        """Extract filament brand from name."""
        if name.startswith("Bambu "):
            return "bambu-lab"
        if name.startswith("Generic "):
            return "generic"
        if name.startswith("PolyLite ") or name.startswith("PolyTerra "):
            return "polymaker"
        if name.startswith("Overture "):
            return "overture"
        if name.startswith("eSUN "):
            return "esun"
        if name.startswith("SUNLU "):
            return "sunlu"

        return "generic"

    def convert_filament(
        self, brand_id: str, name: str, profile: dict
    ) -> Optional[dict]:
        """Convert a single filament profile."""
        resolved = self.resolve_inheritance(profile, brand_id)

        # Parse name
        clean_name = name.replace(" @base", "").replace("@base", "")
        material_type = self.extract_filament_type(clean_name, resolved)
        filament_brand = self.get_filament_brand(clean_name)

        filament_id = slugify(clean_name)

        # Get filament vendor
        vendor = unwrap_array_value(resolved.get("filament_vendor", "Generic"))
        if vendor == "nil" or not vendor:
            vendor = "Generic"

        converted = {
            "$schema": "../schemas/filament.schema.json",
            "id": filament_id,
            "brand": filament_brand,
            "material": material_type,
            "name": clean_name,
            "description": resolved.get("description", f"{clean_name} filament"),
            "source": {
                "origin": "BambuStudio",
                "version": "auto-converted",
                "filament_id": unwrap_array_value(resolved.get("filament_id", "")),
            },
            "physical_properties": self.extract_physical_properties(
                resolved, material_type
            ),
            "temperatures": self.extract_temperatures(resolved),
            "flow": self.extract_flow(resolved),
            "cooling": self.extract_cooling(resolved),
            "retraction": {
                k: v
                for k, v in self.extract_retraction(resolved).items()
                if v is not None
            },
            "multi_material": {
                "prime_volume": parse_numeric(
                    unwrap_array_value(resolved.get("filament_prime_volume", 45))
                )
                or 45,
                "minimal_purge_on_wipe_tower": parse_numeric(
                    unwrap_array_value(
                        resolved.get("filament_minimal_purge_on_wipe_tower", 15)
                    )
                )
                or 15,
            },
            "support_material": {
                "is_support_material": unwrap_array_value(
                    resolved.get("filament_is_support", "0")
                )
                == "1",
                "soluble": unwrap_array_value(resolved.get("filament_soluble", "0"))
                == "1",
            },
            "special_properties": self.extract_special_properties(
                clean_name, material_type, resolved
            ),
            "gcode": {
                "start_gcode": unwrap_array_value(
                    resolved.get("filament_start_gcode", "")
                ),
                "end_gcode": unwrap_array_value(resolved.get("filament_end_gcode", "")),
            },
            "metadata": {
                "author": "BambuStudio Auto-Converter",
                "license": "Apache-2.0",
                "tags": self._get_filament_tags(clean_name, material_type),
            },
        }

        return converted

    def _get_filament_tags(self, name: str, material_type: str) -> list[str]:
        """Generate tags for a filament."""
        tags = [material_type.lower()]
        name_lower = name.lower()

        if "generic" in name_lower:
            tags.append("generic")
        if "bambu" in name_lower:
            tags.append("bambu-lab")

        if material_type in ["PLA"]:
            tags.append("beginner-friendly")
        if material_type in ["ABS", "ASA", "PC"]:
            tags.append("requires-enclosure")
        if "-cf" in name_lower or "-gf" in name_lower:
            tags.append("reinforced")
            tags.append("abrasive")
        if "silk" in name_lower:
            tags.append("decorative")
        if "matte" in name_lower:
            tags.append("matte-finish")

        return tags

    def convert_all(self, output_dir: Path) -> int:
        """Convert all filament profiles."""
        output_path = output_dir / "filaments"
        output_path.mkdir(parents=True, exist_ok=True)

        filaments = self.identify_instantiable_filaments()
        print(f"  Found {len(filaments)} base filament profiles")

        converted_count = 0
        converted_ids = set()

        for brand_id, name, profile in sorted(filaments, key=lambda x: x[1]):
            converted = self.convert_filament(brand_id, name, profile)

            if converted and converted["id"] not in converted_ids:
                output_file = output_path / f"{converted['id']}.json"
                save_json(output_file, converted)
                converted_count += 1
                converted_ids.add(converted["id"])

        return converted_count


# ============================================================================
# Schema Updates
# ============================================================================


def update_schemas(output_dir: Path):
    """Update/create JSON schemas."""
    schemas_dir = output_dir / "schemas"
    schemas_dir.mkdir(parents=True, exist_ok=True)

    # Printer schema
    printer_schema = {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "$id": "slicer/schemas/printer.schema.json",
        "title": "Printer Profile",
        "description": "A 3D printer profile with nested nozzle configurations",
        "type": "object",
        "required": ["id", "brand", "model", "build_volume", "nozzle_configs"],
        "properties": {
            "$schema": {"type": "string"},
            "id": {"type": "string", "pattern": "^[a-z0-9-]+$"},
            "brand": {"type": "string"},
            "model": {"type": "string"},
            "description": {"type": "string"},
            "source": {
                "type": "object",
                "properties": {
                    "origin": {"type": "string"},
                    "version": {"type": "string"},
                    "url": {"type": "string", "format": "uri"},
                },
            },
            "technology": {"type": "string", "enum": ["FFF", "FDM"]},
            "structure": {
                "type": "string",
                "enum": ["cartesian", "corexy", "delta", "polar"],
            },
            "build_volume": {
                "type": "object",
                "required": ["x", "y", "z"],
                "properties": {
                    "x": {"type": "number", "minimum": 0},
                    "y": {"type": "number", "minimum": 0},
                    "z": {"type": "number", "minimum": 0},
                    "origin": {"type": "string", "enum": ["center", "corner"]},
                },
            },
            "default_nozzle_diameter": {"type": "number"},
            "extruder": {"type": "object"},
            "bed": {"type": "object"},
            "enclosure": {"type": "object"},
            "limits": {"type": "object"},
            "features": {"type": "object"},
            "cooling": {"type": "object"},
            "gcode": {"type": "object"},
            "nozzle_configs": {
                "type": "object",
                "additionalProperties": {"$ref": "#/$defs/nozzle_config"},
            },
            "metadata": {"type": "object"},
        },
        "$defs": {
            "nozzle_config": {
                "type": "object",
                "required": ["diameter"],
                "properties": {
                    "diameter": {"type": "number"},
                    "nozzle_type": {"type": "string"},
                    "min_layer_height": {"type": "number"},
                    "max_layer_height": {"type": "number"},
                    "retraction": {"type": "object"},
                    "start_gcode": {"type": "string"},
                },
            },
        },
    }

    save_json(schemas_dir / "printer.schema.json", printer_schema)

    # Filament schema
    filament_schema = {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "$id": "slicer/schemas/filament.schema.json",
        "title": "Filament Profile",
        "description": "A filament material profile",
        "type": "object",
        "required": ["id", "brand", "material", "name"],
        "properties": {
            "$schema": {"type": "string"},
            "id": {"type": "string", "pattern": "^[a-z0-9-]+$"},
            "brand": {"type": "string"},
            "material": {"type": "string"},
            "name": {"type": "string"},
            "description": {"type": "string"},
            "source": {"type": "object"},
            "physical_properties": {"type": "object"},
            "temperatures": {"type": "object"},
            "flow": {"type": "object"},
            "cooling": {"type": "object"},
            "retraction": {"type": "object"},
            "multi_material": {"type": "object"},
            "support_material": {"type": "object"},
            "special_properties": {"type": "object"},
            "gcode": {"type": "object"},
            "metadata": {"type": "object"},
        },
    }

    save_json(schemas_dir / "filament.schema.json", filament_schema)


# ============================================================================
# Main
# ============================================================================


def main():
    parser = argparse.ArgumentParser(description="Convert BambuStudio profiles")
    parser.add_argument(
        "--profiles-dir",
        type=Path,
        default=None,
        help="Path to BambuStudio profiles directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory for converted profiles",
    )
    parser.add_argument(
        "--printers-only",
        action="store_true",
        help="Only convert printer profiles",
    )
    parser.add_argument(
        "--filaments-only",
        action="store_true",
        help="Only convert filament profiles",
    )

    args = parser.parse_args()

    # Default paths
    script_dir = Path(__file__).parent

    if args.profiles_dir:
        profiles_dir = args.profiles_dir
    else:
        # Try common locations
        candidates = [
            script_dir.parent.parent / "BambuStudio" / "resources" / "profiles",
            Path.home()
            / "Code"
            / "3rdParty"
            / "BambuStudio"
            / "resources"
            / "profiles",
        ]
        profiles_dir = None
        for candidate in candidates:
            if candidate.exists():
                profiles_dir = candidate
                break
        if not profiles_dir:
            print("Error: Could not find BambuStudio profiles directory")
            print("Please specify with --profiles-dir")
            sys.exit(1)

    if args.output_dir:
        output_dir = args.output_dir
    else:
        output_dir = script_dir.parent / "data"

    print("=" * 70)
    print("BambuStudio Profile Converter (All Brands)")
    print("=" * 70)
    print(f"\nProfiles source: {profiles_dir}")
    print(f"Output directory: {output_dir}")

    # Discover available brands
    brands = []
    for item in sorted(profiles_dir.iterdir()):
        if item.is_dir() and (item / "machine").exists():
            brands.append(item.name)

    print(f"\nDiscovered brands: {', '.join(brands)}")

    # Update schemas
    print("\n" + "-" * 50)
    print("Updating schemas...")
    update_schemas(output_dir)

    total_printers = 0
    total_filaments = 0

    # Convert printers
    if not args.filaments_only:
        print("\n" + "=" * 50)
        print("CONVERTING PRINTERS")
        print("=" * 50)

        for brand_id in brands:
            print(f"\n[{brand_id}]")
            brand_dir = profiles_dir / brand_id

            converter = PrinterConverter(brand_dir, brand_id)

            print("  Loading profiles...")
            converter.load_all_profiles()

            print("  Identifying printer models...")
            converter.identify_printer_models()

            print("  Categorizing nozzle profiles...")
            converter.categorize_nozzle_profiles()

            print("  Converting...")
            count = converter.convert_all(output_dir)
            total_printers += count
            print(f"  Converted {count} printers")

    # Convert filaments
    if not args.printers_only:
        print("\n" + "=" * 50)
        print("CONVERTING FILAMENTS")
        print("=" * 50)

        converter = FilamentConverter(profiles_dir)

        print("\nLoading all filament profiles...")
        converter.load_all_profiles()

        print("\nConverting...")
        total_filaments = converter.convert_all(output_dir)

    # Summary
    print("\n" + "=" * 70)
    print("CONVERSION COMPLETE")
    print("=" * 70)
    if not args.filaments_only:
        print(f"Printers converted: {total_printers}")
    if not args.printers_only:
        print(f"Filaments converted: {total_filaments}")

    print(f"\nOutput directory: {output_dir}")

    if not args.filaments_only:
        printers_dir = output_dir / "printers"
        if printers_dir.exists():
            print(f"\nPrinter files:")
            for f in sorted(printers_dir.glob("*.json"))[:10]:
                print(f"  - {f.name}")
            printer_count = len(list(printers_dir.glob("*.json")))
            if printer_count > 10:
                print(f"  ... and {printer_count - 10} more")

    if not args.printers_only:
        filaments_dir = output_dir / "filaments"
        if filaments_dir.exists():
            print(f"\nFilament files:")
            for f in sorted(filaments_dir.glob("*.json"))[:10]:
                print(f"  - {f.name}")
            filament_count = len(list(filaments_dir.glob("*.json")))
            if filament_count > 10:
                print(f"  ... and {filament_count - 10} more")


if __name__ == "__main__":
    main()
