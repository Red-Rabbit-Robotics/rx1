#!/usr/bin/env python3

# modified from https://github.com/gstavrinos/calc-inertia/blob/master/calc_inertia_for_urdf.py
# usage: python3 calc_inertia.py existing_urdf.urdf

import os
import sys
import xacro
import collada
from stl import mesh
from urdf_parser_py.urdf import URDF, Mesh, Box, Sphere, Cylinder, Inertial, Inertia
from xml.etree.ElementTree import Element, tostring

def getSTLDimensions(model):
    return model.x.max() - model.x.min(), model.y.max() - model.y.min(), model.z.max() - model.z.min()

def getColladaDimensions(model):
    minx = miny = minz = float("inf")
    maxx = maxy = maxz = float("-inf")
    for tr_vertex in model.geometries[0].primitives[0].vertex[model.geometries[0].primitives[0].vertex_index]:
        for v in tr_vertex:
            maxx = maxx if v[0] <= maxx else v[0]
            maxy = maxy if v[1] <= maxy else v[1]
            maxz = maxz if v[2] <= maxz else v[2]
            minx = minx if v[0] >= minx else v[0]
            miny = miny if v[1] >= miny else v[1]
            minz = minz if v[2] >= minz else v[2]
    return maxx - minx, maxy - miny, maxz - minz

def getInertia(geometry, m, s):
    xx = yy = zz = 0.0
    if type(geometry) == Mesh:
        ROS_VERSION = os.getenv("ROS_VERSION")
        get_pkg_fn = None
        if not ROS_VERSION:
            ROS_VERSION = "2"
        if ROS_VERSION == "1":
            import rospkg
            get_pkg_fn = rospkg.RosPack().get_path
        else:
            import ament_index_python
            get_pkg_fn = ament_index_python.get_package_share_path
        pkg_tag = "package://"
        file_tag = "file://"
        mesh_file = ""
        if geometry.filename.startswith(pkg_tag):
            package, mesh_file = geometry.filename.split(pkg_tag)[1].split(os.sep, 1)
            mesh_file = str(get_pkg_fn(package))+os.sep+mesh_file
        elif geometry.filename.startswith(file_tag):
            mesh_file = geometry.filename.replace(file_tag, "")
        if mesh_file.endswith(".stl"):
            model = mesh.Mesh.from_file(mesh_file)
            x, y, z = getSTLDimensions(model)
        else:
            model = collada.Collada(mesh_file)
            x, y, z = getColladaDimensions(model)
        xx, yy, zz = getBoxInertia(x, y, z, m, s)
    elif type(geometry) == Box:
        x, y, z = geometry.size
        xx, yy, zz = getBoxInertia(x, y, z, m, s)
    elif type(geometry) == Sphere:
        xx, yy, zz = getSphereInertia(geometry.radius, m)
    elif type(geometry) == Cylinder:
        xx, yy, zz = getCylinderInertia(geometry.radius, geometry.length, m)

    return xx, yy, zz

def getBoxInertia(x, y, z, m, s):
    x *= s[0]
    y *= s[1]
    z *= s[2]
    xx = 1./12 * m * (y**2 + z**2)
    yy = 1./12 * m * (x**2 + z**2)
    zz = 1./12 * m * (x**2 + y**2)
    return xx, yy, zz

def getSphereInertia(r, m):
    i = 2./5 * m * r**2
    return i, i, i

def getCylinderInertia(r, h, m):
    xx = yy = 1./12 * m * (3 * r**2 + h**2)
    zz = 1./2 * m * r**2
    return xx, yy, zz

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: script.py <input_urdf_file>")
        sys.exit(1)

    input_urdf_file = sys.argv[1]
    output_urdf_file = "output_with_inertia.urdf"

    # Parse the URDF file
    robot = URDF.from_xml_string(xacro.process_file(input_urdf_file).toprettyxml())

    # Loop through the links and calculate inertia for each
    for link in robot.links:
        link_name = link.name
        inertial = link.inertial
        geometry = None
        mass = None
        scale = [1.0, 1.0, 1.0]

        if inertial:
            mass = inertial.mass
            if link.visual:
                geometry = link.visual.geometry
            elif link.collision:
                geometry = link.collision.geometry

            if geometry and mass:
                if hasattr(geometry, 'scale') and geometry.scale:
                    scale = geometry.scale

                xx, yy, zz = getInertia(geometry, mass, scale)

                # Create a new Inertial object with calculated inertia
                new_inertia = Inertia(ixx=xx, ixy=0.0, ixz=0.0, iyy=yy, iyz=0.0, izz=zz)
                new_inertial = Inertial(mass=mass, inertia=new_inertia, origin=inertial.origin)

                # Update the link's inertial with the new calculated inertia
                link.inertial = new_inertial

    # Convert the updated URDF back to XML and save it
    with open(output_urdf_file, 'w') as f:
        f.write(robot.to_xml_string())

    print(f"New URDF with calculated inertia written to: {output_urdf_file}")

