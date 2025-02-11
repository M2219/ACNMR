from setuptools import setup, find_packages

from setuptools import setup, find_packages

setup(
    name='custom_teleop',
    version='0.1.0',
    packages=find_packages(where="src"),  # Ensures "src/custom_teleop" is included
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    include_package_data=True,
    zip_safe=False,
)
