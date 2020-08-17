import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="anyleaf",
    version="0.1.8",
    author="Anyleaf",
    author_email="anyleaf@anyleaf.org",
    description="Driver for the Anyleaf pH sensor",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/anyleaf/anyleaf_ph",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
        "Topic :: System :: Hardware :: Hardware Drivers ",
    ],
    python_requires=">=3.7",
    license="MIT",
    install_requires=[
        'adafruit-circuitpython-ads1x15>=2.2.2',
        'adafruit-circuitpython-max31865>=2.2.3',
        # 'adafruit-blinka>=5.2.',  # Eables micro/circuitpython support.
        'filterpy>=1.4.5',
        'numpy>=1.19.1',  # For filterpy
        'scipy>=1.5.2',  # For filterpy
    ]
)
