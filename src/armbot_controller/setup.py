from setuptools import setup

package_name = "armbot_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Loi Dinh",
    maintainer_email="loidinh.git@gmail.com",
    description="Package description",
    license="License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slider_control = armbot_controller.slider_control:main",
        ],
    },
)
