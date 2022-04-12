<div align="center">
  <img src=".github/assets/logo.png" height="128">
  <h1>
      ezros
  </h1>
  <h4>Ease your ROS development</h4>
</div>

<p align="center">
    <a href='https://github.com/achillesrasquinha/ezros//actions?query=workflow:"Continuous Integration"'>
      <img src="https://img.shields.io/github/workflow/status/achillesrasquinha/ezros/Continuous Integration?style=flat-square">
    </a>
    <a href="https://coveralls.io/github/achillesrasquinha/ezros">
      <img src="https://img.shields.io/coveralls/github/achillesrasquinha/ezros.svg?style=flat-square">
    </a>
    <a href="https://pypi.org/project/ezros/">
      <img src="https://img.shields.io/pypi/v/ezros.svg?style=flat-square">
    </a>
    <a href="https://pypi.org/project/ezros/">
      <img src="https://img.shields.io/pypi/l/ezros.svg?style=flat-square">
    </a>
    <a href="https://pypi.org/project/ezros/">
		  <img src="https://img.shields.io/pypi/pyversions/ezros.svg?style=flat-square">
	  </a>
    <a href="https://git.io/boilpy">
      <img src="https://img.shields.io/badge/made%20with-boilpy-red.svg?style=flat-square">
    </a>
</p>

### Table of Contents
* [Features](#features)
* [Quick Start](#quick-start)
* [Usage](#usage)
* [License](#license)

### Features
* Python 2.7+ and Python 3.4+ compatible.

### Quick Start

```shell
$ pip install ezros
```

Check out [installation](docs/source/installation.md) for more details.

### Usage

#### Application Interface

```python
# import ezros
import ezros

# create a node...
node = ezros.Node("my_awesome_node")

# subscribe to a topic...
# set message type using the parameter mtype
@node.sub("/my_awesome_topic", mtype = "geometry_msgs.msg.Vector3")
def handle_my_awesome_topic(data):
    # do something with data...
    pass

# handle a publisher...
# set the rate in "Hz" and the queue size.
@node.pub("/my_publish_topic", mtype = "geometry_msgs.msg.Twist", rate = 10, queue_size = 10)
def handle_my_publish_topic(message):
    factor = 10

    # increase by a factor of 10
    message.linear.x *= factor
    message.linear.y *= factor

    # return the message to have it published.
    return message
```


#### Command-Line Interface

```console
$ ezros
Usage: ezros [OPTIONS] COMMAND [ARGS]...

  Ease your ROS development

Options:
  --version   Show the version and exit.
  -h, --help  Show this message and exit.

Commands:
  help     Show this message and exit.
  version  Show version and exit.
```


### License

This repository has been released under the [MIT License](LICENSE).

---

<div align="center">
  Made with ❤️ using <a href="https://git.io/boilpy">boilpy</a>.
</div>
