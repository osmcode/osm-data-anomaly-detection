
# OSM Data Anomaly Detection

Various programs to detect anomalies in OSM data. Many, but not all, of these
are errors that should be fixed.

These are all highly efficient C++ programs that can run through a full planet
PBF file in a few minutes.

## Prerequisites

You need a C++11 compliant compiler. GCC 4.8 and later as well as clang 3.6 and
later are known to work. It also works on modern Visual Studio C++ compilers.

You also need the following libraries:

    Libosmium (>= 2.11.0)
        http://osmcode.org/libosmium
        Debian/Ubuntu: libosmium2-dev

    Protozero (>= 1.4.5)
        This is included in the libosmium repository and might or might not
        have been installed with it. See the libosmium README.
        https://github.com/mapbox/protozero
        Debian/Ubuntu: protozero
        Fedora: protozero-devel

    Utfcpp
        This is included in the libosmium repository and might or might not
        have been installed with it. See the libosmium README.
        http://utfcpp.sourceforge.net/
        Debian/Ubuntu: libutfcpp-dev
        openSUSE: utfcpp
        Fedora: utf8cpp-devel

    bz2lib
        http://www.bzip.org/
        Debian/Ubuntu: libbz2-dev
        Fedora: bzip2-devel
        CentOS: bzip2-devel

    zlib
        http://www.zlib.net/
        Debian/Ubuntu: zlib1g-dev
        openSUSE: zlib-devel
        Fedora: zlib-devel
        CentOS: zlib-devel

    Expat
        http://expat.sourceforge.net/
        Debian/Ubuntu: libexpat1-dev
        openSUSE: libexpat-devel
        Fedora: expat-devel
        CentOS: expat-devel

    cmake
        http://www.cmake.org/
        Debian/Ubuntu: cmake
        openSUSE: cmake
        Fedora: cmake

    GDAL/OGR
        http://gdal.org/
        Debian/Ubuntu: libgdal-dev

## Compiling

A standard CMake setup is used. You can compile like this:

    mkdir build
    cd build
    cmake ..
    make

## Running

All commands take two arguments, the first is the input OSM data file, the
second a directory name where all the output should go. The directory must
exist.

Commands have some options, call with `--help` to see all options. Common
options are:

    -a, --min-age=DAYS      Only include objects at least DAYS days old
    -b, --before=TIMESTAMP  Only include objects changed last before
                            this time (format: yyyy-mm-ddThh:mm:ssZ)
    -h, --help              Print help message
    -q, --quiet             Work quietly

You can not use `--min-age`/`-a` and `--before`/`-b` together.

You can run all commands using the same output directory, they are all using
distinct output file names.

## Results

All programs create

* one or more OSM PBF files with the data of the different anomalies they
  detected,
* an Sqlite file called `stats-*.db` with statistical data, and
* a Spatialite file called `geoms-*.db` containing geometries of the
  data detected (only for some commands).

You can use the script `scripts/collect-stats.sh` to collect the stats from
the various commands into one database called `stats.db`. All stats contain
a timestamp, so you can aggregate stats from, say, daily runs into one large
database.

The timestamp on the stats is the last timestamp of any object in the input
file. This may differ slightly between the various commands, because not all
commands read all object types.

## Commands

### odad-find-colocated-nodes

"Colocated nodes" are nodes that have the exact same location. In OSM that
is usually an error, but it doesn't have to be. There could be two nodes at
the same latitude and longitude but in different heights for instance.

This tool will find all colocated nodes and write them to the output.

Note that the program will create 256 temporary files named `locations_xx.dat`
in the output directory and later remove them. If the program is interrupted
those temporary files might be left around.

The program will need between 1 and 2 GByte RAM for caches.

### odad-find-orphans

"Orphans" are OSM objects (nodes, ways, or relations) that have no tags and
that are not referenced by any other objects. "No tags" in this case also means
objects that have only `source` or `created_by` tags, because they don't say
anything about what an object actually *is*. Orphan objects are always an
error, but, for ways and relations, their members might tell you something
about their intended use.

Do not trust the output of this command when run on an extract! The extract
might not contain all objects referencing the objects in the extract.

### odad-find-unusual-tags

Find "unusual" tags such as empty, very short or long keys, the key "role",
or tag "type=multipolygon" on a node or way.

### odad-find-way-problems

Finds several problems with way geometries:

* Way has no nodes (`no-node`).
* Way has only a single node (`single-node`).
* Way has more than one node, but all nodes are the same (`same-node`).
* Way has two or more references to the same node one right after the other
  (`duplicate-node`).
* Way intersects itself at a place where there is no node (`self-intersection`).
* Way contains a "spike", a segment from node A to node B and, directly after
  that a segment back from node B to node A (`spike`).
* Way contains a duplicate segment, so a connection between two nodes is
  in the way more than once (regardless of the direction of that segment)
  (`duplicate-segment`).

This command needs as input an OSM file with node locations on ways. See the
`osmium [add-locations-to-ways](http://docs.osmcode.org/osmium/latest/osmium-add-locations-to-ways.html)`
command on how to create this.

### odad-find-relation-problems

Finds several problems with relations.

## License

Copyright (C) 2017  Jochen Topf (jochen@topf.org)

This program is available under the GNU GENERAL PUBLIC LICENSE Version 3.
See the file LICENSE.txt for the complete text of the license.

## Author

Jochen Topf (http://jochentopf.com/)

