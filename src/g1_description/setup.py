from setuptools import find_packages, setup
import os
from pathlib import Path

package_name = "g1_description"

def rglob_files(base_dir: str):
    base = Path(base_dir)
    if not base.exists():
        return []
    return [p for p in base.rglob("*") if p.is_file()]

# ----------------------------
# Build data_files properly
# ----------------------------
data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
]

# Copy everything under urdf/ keeping subfolders:
# urdf/meshes, urdf/images, urdf/inspire_hand, etc.
for p in rglob_files("urdf"):
    rel_dir = p.parent.as_posix()              # e.g. "urdf/meshes"
    install_dir = os.path.join("share", package_name, rel_dir)
    data_files.append((install_dir, [p.as_posix()]))

# RViz (optional)
if Path("rviz").is_dir():
    rviz_files = [p.as_posix() for p in rglob_files("rviz") if p.suffix == ".rviz"]
    data_files.append((os.path.join("share", package_name, "rviz"), rviz_files))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jean",
    maintainer_email="icmayoko18@gmail.com",
    description="G1 robot description (URDF/Xacro)",
    license="Apache License 2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
