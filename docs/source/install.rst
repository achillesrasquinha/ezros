.. _install:

### Installation

#### Installation via pip

The recommended way to install **ezros** is via `pip`.

```shell
$ pip install ezros
```

For instructions on installing python and pip see “The Hitchhiker’s Guide to Python” 
[Installation Guides](https://docs.python-guide.org/starting/installation/).

#### Building from source

`ezros` is actively developed on [https://github.com](https://github.com/achillesrasquinha/ezros)
and is always avaliable.

You can clone the base repository with git as follows:

```shell
$ git clone https://github.com/achillesrasquinha/ezros
```

Optionally, you could download the tarball or zipball as follows:

##### For Linux Users

```shell
$ curl -OL https://github.com/achillesrasquinha/tarball/ezros
```

##### For Windows Users

```shell
$ curl -OL https://github.com/achillesrasquinha/zipball/ezros
```

Install necessary dependencies

```shell
$ cd ezros
$ pip install -r requirements.txt
```

Then, go ahead and install ezros in your site-packages as follows:

```shell
$ python setup.py install
```

Check to see if you’ve installed ezros correctly.

```shell
$ ezros --help
```