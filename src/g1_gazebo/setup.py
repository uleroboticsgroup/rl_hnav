
from setuptools import find_packages, setup
import os
from glob import glob

package_name = "g1_gazebo"


def package_files(directory):
    """
    Retourne une liste de tuples (install_dir, [files...]) pour tous les fichiers
    sous 'directory' en conservant l'arborescence relative.
    """
    paths = []
    if not os.path.isdir(directory):
        return paths

    for (path, _, filenames) in os.walk(directory):
        if not filenames:
            continue
        files = [os.path.join(path, f) for f in filenames]
        install_dir = os.path.join("share", package_name, path)
        paths.append((install_dir, files))
    return paths

data_files=[
        # Ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # World files ( .world, .sdf, etc.)
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),
    ]

# ✅ Ajoute models/ en récursif (si le dossier existe)
data_files += package_files("models")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    maintainer='jean',
    maintainer_email='icmayoko18@gmail.com',
    zip_safe=True,
    description="Gazebo simulation for G1 (fake base spawn)",
    license="Apache License 2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
