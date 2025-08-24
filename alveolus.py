import bpy
import bmesh
import math
from copy import deepcopy as dcopy

from mathutils import Vector, Matrix, Quaternion
from math import radians

import itertools

phi = (math.sqrt(5.0) + 1)/2.0
pin = 1.0 / phi

aa = [1, 0, 0]
ab = [phi/2, pin/2, 1/2.0]


# dodecagon center to point
def make_dctp():
    h = 1.0 / (2.0 * math.sin(radians(18)))
    return h

# dodecagon points
def make_dpn(n):
    dpt = Vector((make_dctp(), 0.0, 0.0))
    rot_mat = Matrix.Rotation(radians(36 * n - 18), 3, Vector((0, 0, 1)))
    dpn = rot_mat @ dpt
    return dpn    

# center of right edge of dodecagon
def make_dph01():
    dif = make_dpn(1) - make_dpn(0)
    return make_dpn(0) + dif/2

# pentagon center to point
def make_pctp():
    h = 1.0 / (2.0 * math.sin(radians(36)))
    return h

# pentagon center to edge
def make_pcte():
    h = 1.0 / (2.0 * math.tan(radians(36)))
    return h

# pentagon point to edge
def make_ppte():
    return make_pctp() + make_pcte()

# top pentagon projected onto 0-plane
def make_tzpn(n):
    tzp0 = Vector((make_pctp(), 0, 0))
    rot_mat = Matrix.Rotation(radians(72 * n), 3, Vector((0, 0, 1)))
    tzpn = rot_mat @ tzp0
    return tzpn

# (0-plane pentagon attached to clockwise right edge of dodecahedron).(next ccw point from dodecahedron)
def make_zp0():
    return make_dpn(0)

# (0-plane pentagon attached to clockwise right edge of dodecahedron).(next ccw point from dodecahedron)
def make_zp1():
    return make_dpn(1)

def make_zpoffset():
    return Vector((make_pctp(), 0, 0))

# (0-plane pentagon attached to clockwise right edge of dodecahedron).(next ccw point from dodecahedron)
def make_zp2():
    m = make_zpoffset()
    rot_mat = Matrix.Rotation(radians(72), 3, Vector((0, 0, 1)))
    zp2 = rot_mat @ (make_zp1() - m) + m
    return zp2

# (0-plane pentagon attached to clockwise right edge of dodecahedron).(next ccw point from dodecahedron)
def make_zp3():
    m = make_zpoffset()
    rot_mat = Matrix.Rotation(radians(144), 3, Vector((0, 0, 1)))
    zp3 = rot_mat @ (make_zp1() - m) + m
    return zp3

# (0-plane pentagon attached to clockwise right edge of dodecahedron).(next ccw point from dodecahedron)
def make_zp4():
    m = make_zpoffset()
    rot_mat = Matrix.Rotation(radians(216), 3, Vector((0, 0, 1)))
    zp4 = rot_mat @ (make_zp1() - m) + m
    return zp4

# projected side pentagon length onto 0-plane
def make_psp():
    return (make_tzpn(0) - make_dph01()).length

# side pentagon rotation angle from 0-plane
def make_pra():
    return math.acos(make_psp() / make_ppte())

# rotation axis for 0-plane pentagon
def make_zpra():
    axis = (make_zp1() - make_zp0()).normalized()
    return axis

# rotated pentagon p0
def make_rp0():
    return make_zp0()

# rotated pentagon p1
def make_rp1():
    return make_zp1()

# rotated pentagon p2
def make_rp2():
    rot_axis = make_zpra()
    rot_mat = Matrix.Rotation(make_pra(), 3, rot_axis)
    rp2 = rot_mat @ (make_zp2() - make_zp0()) + make_zp0()
    return rp2

# rotated pentagon p3
def make_rp3():
    rot_axis = make_zpra()
    rot_mat = Matrix.Rotation(make_pra(), 3, rot_axis)
    rp3 = rot_mat @ (make_zp3() - make_zp0()) + make_zp0()
    return rp3
    
# rotated pentagon p4
def make_rp4():
    rot_axis = make_zpra()
    rot_mat = Matrix.Rotation(make_pra(), 3, rot_axis)
    rp4 = rot_mat @ (make_zp4() - make_zp0()) + make_zp0()
    return rp4

def make_rpn(n):
    if n < 5:
        verts = [make_rp0(), make_rp1(), make_rp2(), make_rp3(), make_rp4()]
        rot_mat = Matrix.Rotation(radians(72 * n), 3, Vector((0, 0, 1)))
        verts = [rot_mat @ v for v in verts]
    else:
        rot_mat = Matrix.Rotation(radians(180), 3, Vector((0, 1, 0)))
        verts = [rot_mat @ v for v in make_rpn(n - 5)]
    return verts

# unrotated pentagon
def make_zpo0():
    verts = [make_zp0(), make_zp1(), make_zp2(), make_zp3(), make_zp4()]
    faces = [[0, 1, 2], [0, 2, 3], [0, 3, 4]]
    rp0 = bpy.data.meshes.new("zp0")
    rp0.from_pydata(verts, [], faces)
    rp0.update()
    
    rpo0 = bpy.data.objects.new("zpo0", rp0)
    bpy.context.collection.objects.link(rpo0)

# rotated pentagon
def make_rpvsn(n):
    verts = make_rpn(n)
    return verts
    
# dodecagon
def make_do():
    verts = [make_dpn(n) for n in range(10)]
    faces = [[0, a, a + 1] for a in range(1, 9)]
    d = bpy.data.meshes.new("d")
    d.from_pydata(verts, [], faces)
    d.update()
    
    do = bpy.data.objects.new("do", d)
    bpy.context.collection.objects.link(do)

def make_tpvs():
    rp3 = make_rp3()
    verts = []
    for n in range(0, 5):
        rot_mat = Matrix.Rotation(radians(72 * n), 3, Vector((0, 0, 1)))
        verts.append(rot_mat @ rp3)
    return verts
    
def make_bpvs():
    rot_mat = Matrix.Rotation(radians(180), 3, Vector((0, 1, 0)))
    verts = [rot_mat @ v for v in make_tpvs()]
    return verts
    
inch = 2.54/100
w = 5.5 * inch

def prv(name, indent, vector):
    print (f"{' '*indent}{name}: ({vector[0]}, {vector[1]}, {vector[2]}) - {vector.length}")

def pre(name, indent, edge):
    vdiff = edge[1] - edge[0]
    print (f"{' '*indent}{name}: ({edge[0][0]}, {edge[0][1]}, {edge[0][2]}); ({edge[1][0]}, {edge[1][1]}, {edge[1][2]}) len: {vdiff.length}")
    
def make_pentagon(mesh_name, obj_name, verts):
    faces = [[0, a, a + 1] for a in range(1, 4)]
    tzp = bpy.data.meshes.new(mesh_name)
    tzp.from_pydata(verts, [], faces)
    tzp.update()
    
    tzpo = bpy.data.objects.new(obj_name, tzp)
    bpy.context.collection.objects.link(tzpo)
    
def make_cylinder(mesh_name, obj_name, verts, radius):
    delta = verts[1] - verts[0]
    midpoint = (verts[0] + verts[1]) / 2
    #phi = math.atan2(delta[1], delta[0])
    #theta = math.acos(delta[2] / delta.length)
    rotation_matrix = Vector((0,0,1)).rotation_difference(delta.normalized()).to_matrix()
    cyl_mesh = bpy.ops.mesh.primitive_cylinder_add(
        radius=radius, 
        depth=delta.length, 
        location=midpoint, 
        rotation=rotation_matrix.to_euler(),
        vertices=16)
    pre("hinge", 4, verts)

#def make_cyl2(name, verts, radius):
#    delta = verts[1] - verts[0]
#    bpy.ops.curve.

def make_strut(verts, normal):
    mesh = bpy.data.meshes.new("StrutMesh")
    obj = bpy.data.objects.new("Strut", mesh)
    bpy.context.collection.objects.link(obj)
    
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    
    bm = bmesh.new()
    bm.from_mesh(mesh)

    verts = [bm.verts.new(v) for v in verts]
    
    bm.faces.new(verts)
    bm.to_mesh(mesh)
    bm.free()
    
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.extrude_region_move(
        TRANSFORM_OT_translate={"value": normal}
    )
    
    bpy.ops.object.mode_set(mode='OBJECT')
    obj.select_set(False)
    
    bpy.context.view_layer.objects.active = None
    return
    rot_obj = obj.copy()
    rot_obj.data = obj.data.copy()
    bpy.context.collection.objects.link(rot_obj)
    rot_obj.rotation_mode = 'QUATERNION'
    rot_obj.rotation_quaternion = rot_quat @ rot_obj.rotation_quaternion
    bpy.context.view_layer.objects.active = None
    
    

def make_cross(edge, strut_length, centerpoint):
    pre("interior edge", 2, edge)
    #print (f"  Edge: v0: {edge[0]};    v1: {edge[1]}")
    prv("centerpoint", 2, centerpoint)
    #print (f"  center: {centerpoint}")
    midpoint = (edge[0] + edge[1]) / 2
    prv("midpoint", 2, midpoint)
    #print (f"  midpoint: {midpoint}")
    #d = edge[0].length     # dist from 0
    #v0n = (edge[0] - centerpoint).normalized()
    #v1n = (edge[1] - centerpoint).normalized()
    v0n = (edge[0]).normalized()
    v1n = (edge[1]).normalized()
    theta = math.acos(Vector.dot(v0n, v1n))
    #print (f"  theta: {theta/math.pi}pi")
    axis = Vector.cross(v0n, v1n).normalized()
    s = (edge[1] - midpoint).length
    #print (f"  s: {s}")
    phi = math.pi / 2 - (theta / 2)
    #print (f"  phi: {phi/math.pi}pi")
    k = math.sin(phi) * s
    #print (f"  k: {k}")
    assert(k < strut_length)
    lam = math.acos(k / strut_length)
    #print (f"  lam: {lam}")
    lower_hinge1 = -midpoint.normalized() * strut_length
    rot_mat_a = Matrix.Rotation(-(phi - lam), 3, axis)
    lower_hinge1 = rot_mat_a @ lower_hinge1
    rot_mat_b = Matrix.Rotation((radians(180) - theta), 3, axis)
    upper_hinge0 = rot_mat_b @ lower_hinge1
    lower_hinge1 += midpoint - axis * inch*1.625
    upper_hinge0 += midpoint - axis * inch*1.625
    midpoint_pos = midpoint - axis * inch*1.625
    
    lower_strut_axis = (lower_hinge1 - midpoint_pos).normalized()
    upper_strut_axis = (upper_hinge0 - midpoint_pos).normalized()
    crotch_axis = (((lower_hinge1 + upper_hinge0) / 2) - midpoint_pos).normalized()
    wt = w / 2 / math.sin((math.pi - theta) / 2)
    crotch_point = midpoint_pos + crotch_axis * wt
    shoulder_point = midpoint_pos - crotch_axis * wt
    lower_width_axis = Vector.cross(lower_strut_axis, axis)
    upper_width_axis = Vector.cross(upper_strut_axis, axis)

    rot_mat_c = Matrix.Rotation(radians(180), 4, midpoint.normalized())
    
    verts0 = [
        lower_hinge1 + lower_strut_axis * w / 2 + lower_width_axis * w / 4, 
        lower_hinge1 + lower_strut_axis * w / 4 + lower_width_axis * w / 2, 
        shoulder_point,
        upper_hinge0 + upper_strut_axis * w / 4 - upper_width_axis * w / 2,
        upper_hinge0 + upper_strut_axis * w / 2 - upper_width_axis * w / 4,
        
        upper_hinge0 + upper_strut_axis * w / 2 + upper_width_axis * w / 4,
        upper_hinge0 + upper_strut_axis * w / 4 + upper_width_axis * w / 2,
        crotch_point,
        lower_hinge1 + lower_strut_axis * w / 4 - lower_width_axis * w / 2,
        lower_hinge1 + lower_strut_axis * w / 2 - lower_width_axis * w / 4]
    verts1 = [rot_mat_c @ v for v in verts0]
    
    verts0 = [v + centerpoint for v in verts0]
    verts1 = [v + centerpoint for v in verts1]
    make_strut(verts0, axis * inch * 1.5)
    make_strut(verts1, -axis * inch * 1.5)
    
    lower_hinge0 = rot_mat_c @ lower_hinge1
    upper_hinge1 = rot_mat_c @ upper_hinge0
    
    make_cylinder("hinge_mesh", "hinge", [lower_hinge1 - axis * 0.25 * inch + centerpoint, lower_hinge1 + axis * 1.625 * inch + centerpoint], 2 * inch)
    make_cylinder("hinge_mesh", "hinge", [upper_hinge0 - axis * 0.25 * inch + centerpoint, upper_hinge0 + axis * 1.625 * inch + centerpoint], 2 * inch)
    make_cylinder("hinge_mesh", "hinge", [lower_hinge0 + axis * 0.25 * inch + centerpoint, lower_hinge0 - axis * 1.625 * inch + centerpoint], 3 * inch)
    make_cylinder("hinge_mesh", "hinge", [upper_hinge1 + axis * 0.25 * inch + centerpoint, upper_hinge1 - axis * 1.625 * inch + centerpoint], 2 * inch)
    make_cylinder("hinge_mesh", "hinge", [midpoint_pos - axis * 0.25 * inch + centerpoint, midpoint_pos + axis * 3.5 * inch + centerpoint], 1 * inch)
    

# edge's verts [0, 1] must be equidistant from the center (0, 0, 0)        
def make_crosses(edge, num_crosses, strut_length, case_pitch):
    assert(num_crosses > 0)
    pre("Polygon edge", 0, edge)
    v0n = edge[0].normalized()
    v1n = edge[1].normalized()
    theta = math.acos(Vector.dot(v0n, v1n))
    axis = Vector.cross(v0n, v1n).normalized()
    prv("axis", 0, axis)
    midpoint = (edge[1] + edge[0]) / 2
    mid_axis = midpoint.normalized()
    center_offset = case_pitch / 2 / math.sin(theta / 2)
    center = mid_axis * center_offset
    v0cross = Vector.cross(v0n, -axis).normalized()
    v1cross = Vector.cross(v1n, axis).normalized()
    
    print (f"v0cross: {v0cross};    v1cross: {v1cross}")
    
    edge[0] -= center
    edge[1] -= center
    edge[0] += v0cross * case_pitch / 2
    edge[1] += v1cross * case_pitch / 2
    sub_verts = []
    sub_verts.append(edge[0])
    for sub_edge in range(num_crosses - 1):
        rot_mat = Matrix.Rotation(theta / num_crosses, 3, axis)
        sub_verts.append(rot_mat @ sub_verts[-1])
    sub_verts.append(edge[1])
    for sub_edge in range(len(sub_verts) - 1):
        #make_cross((sub_verts[sub_edge] + center, sub_verts[sub_edge + 1] + center), 
        #    strut_length, center)
        make_cross((sub_verts[sub_edge], sub_verts[sub_edge + 1]), 
            strut_length, center)
    
def c(v: list):
    return dcopy(v)
    
class Hedron:
    def __init__(self, location, z_angle, radius):
        self.hemigon_verts = []
        self.mid_upper_verts = []
        self.top_verts = make_tpvs()
        self.mid_lower_verts = []
        self.bottom_verts = make_bpvs()

        self.edges_hemigon = []
        self.edges_level1_upper = []
        self.edges_level2_upper = []
        self.edges_top_pent = []
        self.edges_level1_lower = []
        self.edges_level2_lower = []
        self.edges_bottom_pent = []
        
        for n in range(5):
            upper_pent = make_rpn(n)
            lower_pent = make_rpn(n + 5)
            self.hemigon_verts.append(upper_pent[0])
            self.hemigon_verts.append(upper_pent[1])
            self.mid_upper_verts.append(upper_pent[2])
            self.mid_lower_verts.append(lower_pent[2])
            self.top_verts.append(upper_pent[3])
            self.bottom_verts.append(upper_pent[3])
        
        self.hemigon_verts =    [v * radius / v.length for v in self.hemigon_verts]
        self.mid_upper_verts =  [v * radius / v.length for v in self.mid_upper_verts]
        self.top_verts =        [v * radius / v.length for v in self.top_verts]
        self.mid_lower_verts =  [v * radius / v.length for v in self.mid_lower_verts]
        self.bottom_verts =     [v * radius / v.length for v in self.bottom_verts]
            
        for n in range(10):
            self.edges_hemigon.append([c(self.hemigon_verts[n]), c(self.hemigon_verts[(n+1)%10])])
            
        for n in range(5):
            self.edges_level1_upper.append([c(self.hemigon_verts[(n*2)%10]), c(self.mid_upper_verts[(n-1)%5])])
            self.edges_level1_upper.append([c(self.hemigon_verts[(n*2+1)%10]), c(self.mid_upper_verts[(n)%5])])
            self.edges_level2_upper.append([c(self.mid_upper_verts[n]), c(self.top_verts[(n+1)%5])])
            self.edges_level2_upper.append([c(self.mid_upper_verts[(n+1)%5]), c(self.top_verts[(n+1)%5])])
            self.edges_top_pent.append([c(self.top_verts[n]), c(self.top_verts[(n+1)%5])])

            self.edges_level1_lower.append([c(self.hemigon_verts[(n*2)%10]), c(self.mid_lower_verts[(7-n)%5])])
            self.edges_level1_lower.append([c(self.hemigon_verts[(n*2+1)%10]), c(self.mid_lower_verts[(7-n)%5])])
            self.edges_level2_lower.append([c(self.mid_lower_verts[n]), c(self.bottom_verts[(n+1)%5])])
            self.edges_level2_lower.append([c(self.mid_lower_verts[(n+1)%5]), c(self.bottom_verts[(n+1)%5])])
            self.edges_bottom_pent.append([c(self.bottom_verts[n]), c(self.bottom_verts[(n+1)%5])])


    def make_pents(self):
        for n in range(5):
            make_pentagon(f"upper_pent_mesh", f"upper_pent", [
                self.hemigon_verts[n*2],
                self.hemigon_verts[n*2 + 1],
                self.mid_upper_verts[n],
                self.top_verts[n],
                self.mid_upper_verts[(n - 1 + 5) % 5]])
        for n in range(5):
            make_pentagon(f"lower_pent_mesh", f"lower_pent", [
                self.hemigon_verts[(n*2 + 2) % 10],
                self.hemigon_verts[(n*2 + 1) % 10],
                self.mid_lower_verts[(7 - n) % 5],
                self.bottom_verts[(7 - n) % 5],
                self.mid_lower_verts[(6 - n) % 5]])
        make_pentagon(f"top_pent_mesh", "top_pent", self.top_verts)
        make_pentagon(f"bottom_pent_mesh", "bottom_pent", self.bottom_verts)

    def iterate_edges(self):
        return itertools.chain(self.edges_hemigon, self.edges_level1_upper, self.edges_level2_upper, 
            self.edges_top_pent, self.edges_level1_lower, self.edges_level2_lower, self.edges_bottom_pent)
    
    def report_edges(self):
        i = 0
        for edge in self.iterate_edges():
            pre(f"Edge{i}", 0, edge)
            i+=1
    
    def make_cyls(self):
        for edge in self.iterate_edges():
            make_cylinder(f"cyl_mesh", f"cyl", edge, 0.02)
    
    def make_crosses(self, num_segments_per_edge, strut_length, case_pitch):
        i = 0
        for edge in self.iterate_edges():
            pre(f"Edge{i}", 0, edge)
            make_crosses(edge, num_segments_per_edge, strut_length, case_pitch)
            #i += 1
            #if i == 3:
            #    return
            

tau = math.pi * 2
theta = tau / 10

def get_extents(num_struts, strut_length, case_pitch, alpha_min, alpha_max):
    # lol here we go
    theta_t = theta / num_struts
    theta_tt = theta_t / 2
    rd = case_pitch / 2 / math.tan(theta / 2)
    beta = tau / 4 + theta_tt
    
    def make_len(alpha):
        gamma = tau / 2 - beta - alpha
        et2 = strut_length * math.sin(gamma) / math.sin(beta)
        rt = et2 / math.sin(theta_tt)
        r = rt + rd
        return r
        
    long_r = make_len(alpha_min)
    short_r = make_len(alpha_max)
    
    print (f"long_r: {long_r}; short_r: {short_r}")
    
    return (long_r, short_r)
        

def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)       

def make_icosidodecahedron():
    making_extents_1 = False
    making_extents_2 = False
    making_extents_3 = True
    
    if making_extents_3:
        num_struts = 3
        strut_length = 46*inch
        case_pitch = 12*inch
        alpha_min = radians(10)
        alpha_max = tau / 4 - theta / num_struts - radians(5)
        long_r, short_r = get_extents(3, strut_length, case_pitch, alpha_min, alpha_max)
        h = Hedron((0, 0, 0), 0, long_r)
        h.make_crosses(num_struts, strut_length, case_pitch)
        h = Hedron((0, 0, 0), 0, short_r)
        h.make_crosses(num_struts, strut_length, case_pitch)

    if making_extents_2:
        num_struts = 2
        strut_length = 74*inch
        case_pitch = 12*inch
        alpha_min = radians(10)
        alpha_max = tau / 4 - theta / num_struts - radians(5)
        long_r, short_r = get_extents(num_struts, strut_length, case_pitch, alpha_min, alpha_max)
        h = Hedron((0, 0, 0), 0, long_r)
        h.make_crosses(num_struts, strut_length, case_pitch)
        h = Hedron((0, 0, 0), 0, short_r)
        h.make_crosses(num_struts, strut_length, case_pitch)

    
    if making_extents_1:
        num_struts = 1
        strut_length = 74*inch
        case_pitch = 12*inch
        alpha_min = radians(10)
        alpha_max = tau / 4 - theta / num_struts - radians(5)
        long_r, short_r = get_extents(num_struts, strut_length, case_pitch, alpha_min, alpha_max)
        h = Hedron((0, 0, 0), 0, long_r)
        h.make_crosses(num_struts, strut_length, case_pitch)
        h = Hedron((0, 0, 0), 0, short_r)
        h.make_crosses(num_struts, strut_length, case_pitch)
        

    #h = Hedron((0, 0, 0), 0, phi)
    #h.make_pents()
    #h.make_cyls()

#reset_scene()
make_icosidodecahedron()
print ("---------------------------------------------")
