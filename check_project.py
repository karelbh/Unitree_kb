#!/usr/bin/env python3
"""
Ověřovací skript pro projekt Unitree_kb.

Kontroluje, že projekt je správně připravený pro práci doma i v pískovně.
Nepoužívá MuJoCo viewer, jen ověřuje konfiguraci a dostupnost knihoven.
"""

import sys
import os
from pathlib import Path


def check_python_environment():
    """Ověří Python environment."""
    print("=" * 60)
    print("PYTHON PROSTŘEDÍ")
    print("=" * 60)
    print(f"Executable: {sys.executable}")
    print(f"Verze: {sys.version}")
    print()


def check_imports():
    """Ověří dostupnost důležitých knihoven."""
    print("=" * 60)
    print("KNIHOVNY")
    print("=" * 60)
    
    libraries = {
        "mujoco": "MuJoCo simulátor",
        "cv2": "OpenCV pro zpracování obrazu",
        "numpy": "NumPy pro numeriku",
    }
    
    all_ok = True
    for lib_name, lib_description in libraries.items():
        try:
            __import__(lib_name)
            print(f"✓ {lib_name:15} - {lib_description}")
        except ImportError:
            print(f"✗ {lib_name:15} - CHYBÍ! ({lib_description})")
            all_ok = False
    
    print()
    return all_ok


def check_model_loading():
    """Načte model G1 a ověří jeho parametry."""
    print("=" * 60)
    print("MODEL G1")
    print("=" * 60)
    
    try:
        from unitree_paths import load_g1_model
        print("✓ Modul unitree_paths importován úspěšně")
    except ImportError as e:
        print(f"✗ Nelze importovat unitree_paths: {e}")
        return False, None
    
    try:
        model = load_g1_model()
        print("✓ Model G1 načten úspěšně")
    except FileNotFoundError as e:
        print(f"✗ Model G1 nenalezen: {e}")
        return False, None
    except Exception as e:
        print(f"✗ Chyba při načítání modelu: {e}")
        return False, None
    
    print()
    return True, model


def check_model_parameters(model):
    """Ověří parametry modelu."""
    print("=" * 60)
    print("PARAMETRY MODELU")
    print("=" * 60)
    
    num_joints = model.njnt
    num_actuators = model.nu
    
    print(f"Počet kloubů: {num_joints}")
    print(f"Počet aktuátorů: {num_actuators}")
    print()
    
    # Ověření minimálních hodnot
    min_joints = 25
    min_actuators = 20
    
    all_ok = True
    
    if num_joints >= min_joints:
        print(f"✓ Kloubů {num_joints} ≥ {min_joints} (OK)")
    else:
        print(f"✗ Kloubů {num_joints} < {min_joints} (CHYBA)")
        all_ok = False
    
    if num_actuators >= min_actuators:
        print(f"✓ Aktuátorů {num_actuators} ≥ {min_actuators} (OK)")
    else:
        print(f"✗ Aktuátorů {num_actuators} < {min_actuators} (CHYBA)")
        all_ok = False
    
    print()
    return all_ok


def check_hardcoded_paths():
    """Prohledá .py soubory na pevné cesty."""
    print("=" * 60)
    print("VYHLEDÁVÁNÍ PEVNÝCH CEST")
    print("=" * 60)
    
    hardcoded_paths = [
        r"F:\Unitree",
        r"U:\Unitree",
        r"C:\Users\bohm",
        r"C:\Users\Geronimo",
    ]
    
    project_root = Path(__file__).parent
    py_files = list(project_root.glob("*.py"))
    
    # Ignoruj sám check_project.py a unitree_paths.py (jsou to helper skripty s hardcoded cestami)
    ignore_files = {"check_project.py", "unitree_paths.py"}
    
    warnings = []
    
    for py_file in py_files:
        if py_file.name in ignore_files:
            continue
        
        try:
            with open(py_file, "r", encoding="utf-8") as f:
                content = f.read()
                for hardcoded in hardcoded_paths:
                    if hardcoded in content:
                        warnings.append(f"{py_file.name}: obsahuje cestu '{hardcoded}'")
        except Exception as e:
            print(f"⚠ Nelze přečíst {py_file.name}: {e}")
    
    if warnings:
        print("⚠ ZJIŠTĚNY PEVNÉ CESTY:")
        for warning in warnings:
            print(f"  {warning}")
        print()
        return False
    else:
        print("✓ Žádné pevné cesty nenalezeny")
        print()
        return True


def main():
    """Hlavní funkce."""
    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║" + " " * 58 + "║")
    print("║" + "  KONTROLA PROJEKTU UNITREE_KB".center(58) + "║")
    print("║" + " " * 58 + "║")
    print("╚" + "═" * 58 + "╝")
    print("\n")
    
    results = {
        "python": True,
        "imports": True,
        "model": True,
        "parameters": True,
        "paths": True,
    }
    
    # Kontrola Python prostředí
    check_python_environment()
    
    # Kontrola knihoven
    results["imports"] = check_imports()
    if not results["imports"]:
        print("⚠ Některé knihovny chybí. Instalace:")
        print("  pip install mujoco opencv-python numpy")
        print()
    
    # Kontrola modelu
    model_ok, model = check_model_loading()
    results["model"] = model_ok
    
    if model_ok:
        # Kontrola parametrů modelu
        results["parameters"] = check_model_parameters(model)
    
    # Kontrola pevných cest
    results["paths"] = check_hardcoded_paths()
    
    # Výsledek
    print("=" * 60)
    print("SOUHRN")
    print("=" * 60)
    
    all_ok = all(results.values())
    
    if all_ok:
        print("\n")
        print("╔" + "═" * 58 + "╗")
        print("║" + " " * 58 + "║")
        print("║" + "  KONTROLA PROJEKTU OK".center(58) + "║")
        print("║" + " " * 58 + "║")
        print("╚" + "═" * 58 + "╝")
        print("\n")
        return 0
    else:
        print("\n⚠ Projekt má problémy:")
        if not results["python"]:
            print("  - Python prostředí není v pořádku")
        if not results["imports"]:
            print("  - Chybí některé knihovny")
        if not results["model"]:
            print("  - Model G1 se nepodařilo načíst")
        if not results["parameters"]:
            print("  - Model nemá dostatek kloubů/aktuátorů")
        if not results["paths"]:
            print("  - Zdrojový kód obsahuje pevné cesty")
        print()
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
