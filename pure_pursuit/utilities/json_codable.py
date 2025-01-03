#
# json_codable.py
# pure-pursuit
#
# Created by Christian Bator on 01/02/2025
#

from typing import Any, Self, Protocol

#
# JSON
#
JSON = dict[str, Any]

#
# JSONDecodable
#
class JSONDecodable(Protocol):

    @classmethod
    def decode(cls, json_data: JSON) -> Self:
        ...
