

# imports - module imports
from ezros.exception import (
    EzrosError
)

# imports - test imports
import pytest

def test_ezros_error():
    with pytest.raises(EzrosError):
        raise EzrosError