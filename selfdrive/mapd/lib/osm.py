import json
import time

import numpy as np
import overpy
import requests

from selfdrive.mapd.lib.geo import R


OVERPASS_URLS = [
  "https://z.overpass-api.de/api/interpreter",
  "https://overpass-api.de/api/interpreter",
  "https://lz4.overpass-api.de/api/interpreter",
  "https://overpass.kumi.systems/api/interpreter",
]

USER_AGENT = "dragonpilot-mapd/0.8.13"

HEADERS = {
  "User-Agent": USER_AGENT,
  "Accept": "application/json",
  "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
}


def create_way(way_id, node_ids, from_way):
  """
  Creates an OSM Way with the given way_id and list of node_ids,
  copying attributes and tags from from_way.
  """
  return overpy.Way(
    way_id,
    node_ids=node_ids,
    attributes={},
    result=from_way._result,
    tags=from_way.tags,
  )


class OSM():
  def __init__(self):
    self.parser = overpy.Overpass()
    self.last_fail_time = 0.0

  def fetch_road_ways_around_location(self, lat, lon, radius):
    # Avoid hammering Overpass if all servers just failed.
    now = time.monotonic()
    if now - self.last_fail_time < 60.0:
      return []

    bbox_angle = np.degrees(radius / R)

    south = lat - bbox_angle
    west = lon - bbox_angle
    north = lat + bbox_angle
    east = lon + bbox_angle

    q = """
[out:json][timeout:25];
(
  way["highway"]["highway"!~"^(footway|path|corridor|bridleway|steps|cycleway|construction|bus_guideway|escape|service|track)$"](%f,%f,%f,%f);
);
(._;>;);
out body;
""" % (south, west, north, east)

    for url in OVERPASS_URLS:
      try:
        print("OSM: querying %s radius=%s bbox=%f,%f,%f,%f" % (url, radius, south, west, north, east), flush=True)

        r = requests.post(
          url,
          data={"data": q},
          headers=HEADERS,
          timeout=30,
        )

        if r.status_code != 200:
          print("OSM: %s returned HTTP %s: %s" % (url, r.status_code, r.text[:200]), flush=True)
          continue

        result = self.parser.parse_json(r.text)
        ways = result.ways

        print("OSM: got %d ways from %s" % (len(ways), url), flush=True)
        return ways

      except BaseException as e:
        print("OSM: query failed on %s: %s" % (url, e), flush=True)

    self.last_fail_time = time.monotonic()
    print("OSM: all Overpass queries failed; backing off 60s", flush=True)
    return []
