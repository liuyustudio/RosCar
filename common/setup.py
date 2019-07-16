import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ros-car-common-library",
    version="0.1.0",
    author="Liu Yu",
    author_email="source@liuyu.com",
    description="Common library of ROS Car project",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/liuyustudio/RosCar",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
