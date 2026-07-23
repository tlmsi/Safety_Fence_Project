from setuptools import find_packages, setup

package_name = 'sorting_cell_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osama',
    maintainer_email='osama@example.com',
    description='RGB perception for the autonomous sorting cell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'color_sort_detector = '
                'sorting_cell_perception.color_sort_detector:main'
            ),
        ],
    },
)
