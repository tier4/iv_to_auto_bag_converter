from setuptools import setup

package_name = "iv_to_auto_bag_converter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "config/topic_list.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryu Yamamoto, Hayato Mizushima",
    maintainer_email="ryu.yamamoto@tier4.jp, hayato-m126@users.noreply.github.com",
    description="convert .iv bag to .auto",
    license="Apache V2",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "convert = " + package_name + ".converter:main",
            "convert_multi = " + package_name + ".sequential_converter:main",
        ],
    },
)
