from setuptools import setup, find_packages

pkg = find_packages()

setup(
   name='bmp',
   version='0.0.1',
   description='bmp',
   author='Kanghyun Kim',
   author_email='kh11kim@kaist.ac.kr',
   packages=find_packages(),
   install_requires=["numpy", "scipy", "pybullet", "trimesh"], #external packages as dependencies
)