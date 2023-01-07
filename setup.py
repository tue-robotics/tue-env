import os.path

from setuptools import find_packages
from setuptools import setup

with open(os.path.join(os.path.dirname(__file__), "VERSION"), "r") as f:
    version = f.readline().strip()

print(find_packages(where="src"))

setup(
    name="tue_env",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={"tue_get": ["src/tue_get/resources/*"]},
    include_package_data=True,
    version=version,
    author="Matthijs van der Burgh",
    author_email="MatthijsBurgh@outlook.com",
    maintainer="Matthijs van der Burgh",
    maintainer_email="MatthijsBurgh@outlook.com",
    url="https://github.com/tue-robotics/tue-env",
    keywords=["catkin"],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "Programming Language :: Python",
    ],
    description="Plugin for catkin_tools to enable building workspace documentation.",
    # entry_points={
    #     "catkin_tools.commands.catkin.verbs": [
    #         "document = catkin_tools_document:description",
    #     ],
    #     "catkin_tools.spaces": [
    #         "docs = catkin_tools_document.spaces.docs:description",
    #     ],
    # },
    python_version=">=3.8",
    install_requires=[
        "termcolor",
        "pyyaml",
    ],
)
