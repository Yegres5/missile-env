from setuptools import setup

setup(
    name="missile_env",
    version='0.0.1',
    desctiption="Private package for guided missile model",
    url='git@github.com:Yegres5/missile-env.git',
    author='Evgeny Sazhnev',
    author_email='yegres98@gmail.com',
    license='unlicensed',
    packages=['missile_env'],
    install_requiers=['gym, numpy']
)
