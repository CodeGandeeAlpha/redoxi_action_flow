from setuptools import find_packages, setup
import glob

package_name = "redoxi_common_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            # this will allow the package to be imported in launch files
            "lib/" + package_name,
            [
                f
                for f in glob.glob(f"{package_name}/**/*.py", recursive=True)
                if not f.endswith("__init__.py")
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="igamenovoer",
    maintainer_email="igamenovoer@xx.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
