from setuptools import find_packages, setup

package_name = 'packprojet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miossec',
    maintainer_email='miossec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'simulateur_temp = packprojet.simulateur_temp:main',
        	'controle_temp = packprojet.controle_temp:main',
        	'led_node = packprojet.led_node:main',
        	'bouton_poussoir = packprojet.bouton_poussoir:main',
        	'fenetre_node = packprojet.fenetre_node:main',
        ],
    },
)
