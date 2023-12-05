import sys,os
import ctypes
import pdb
import numpy as np

'''
so_path = '/home/step/work/Robot/gpd/build/'
sys.path.append(so_path)
#pdb.set_trace()
os.chdir(so_path)
'''

'''
extern "C" struct Grasp {
  double *pos;
  double *orient;
  double *sample;
  double score;
  bool label;
  int *image;
};
'''
class StructGrasp(ctypes.Structure):
    _fields_ = [('pos_p',ctypes.POINTER(ctypes.c_double)*1),
                ('orient_p',ctypes.POINTER(ctypes.c_double)*1),
                ('rotate_p',ctypes.POINTER(ctypes.c_double)*1),
                ('sample_p',ctypes.POINTER(ctypes.c_double)*1),
                ('score',ctypes.c_double),
                ('label',ctypes.c_bool),
                ('image_p',ctypes.POINTER(ctypes.c_int32))
               ]


class CGPD(object):
    def __init__(self):
        # 加载SO文件
        #self.lib = ctypes.CDLL('./gpd/libgpd_c.so')
        self.lib = ctypes.CDLL('../build/libgpd_c.so')
        self.hand_num = 0
        self.grasps_out_p = ctypes.pointer(StructGrasp())

        self.grasps_hand = []

        '''
        extern "C" int detectGraspsInFile(char *config_filename, char *pcd_filename,
                                        char *normals_filename, float *view_points,
                                        int num_view_points,
                                        struct Grasp **grasps_out) {
        '''
        # 指定函数参数和返回值类型
        self.lib.detectGraspsInFile.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                        ctypes.c_char_p, ctypes.POINTER(ctypes.c_float),
                                        ctypes.c_int32,
                                        ctypes.POINTER(ctypes.POINTER(StructGrasp))
                                        ]
        self.lib.detectGraspsInFile.restype = ctypes.c_int

        '''
        extern "C" int detectGraspsInCloud(char *config_filename, float *points,
                                        int *camera_index, float *view_points,
                                        int size, int num_view_points,
                                        struct Grasp **grasps_out) {
        '''
        # 指定函数参数和返回值类型
        self.lib.detectGraspsInCloud.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float),
                                        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_float),
                                        ctypes.c_int32,ctypes.c_int32,
                                        ctypes.POINTER(ctypes.POINTER(StructGrasp))
                                        ]
        self.lib.detectGraspsInCloud.restype = ctypes.c_int

    def detectGraspsInFile(self, config_filename, pcd_filename, normals_filename):
        num_view_points = 1
        # this is 2d array
        #view_points = (ctypes.c_float * 3 * num_view_points)()
        # this is 1d array
        size = 3 * num_view_points
        view_points = (ctypes.c_float * size)(0.0,0.0,0.0)

        self.hand_num = self.lib.detectGraspsInFile(config_filename,pcd_filename,normals_filename,
                                  view_points,num_view_points,
                                  ctypes.byref(self.grasps_out_p))

        self.grasps_hand = []
        for i in range(self.hand_num):
            print('\ngrasp ',i)

            hdict = {}
            hdict['pos']    = np.array([self.grasps_out_p[i].pos_p[0][0],self.grasps_out_p[i].pos_p[0][1],self.grasps_out_p[i].pos_p[0][2]])
            hdict['orient'] = np.array([self.grasps_out_p[i].orient_p[0][0],self.grasps_out_p[i].orient_p[0][1],
                               self.grasps_out_p[i].orient_p[0][2],self.grasps_out_p[i].orient_p[0][3]])
            hdict['rotate'] = np.array([[self.grasps_out_p[i].rotate_p[0][0],self.grasps_out_p[i].rotate_p[0][1],self.grasps_out_p[i].rotate_p[0][2]],
                               [self.grasps_out_p[i].rotate_p[0][3],self.grasps_out_p[i].rotate_p[0][4],self.grasps_out_p[i].rotate_p[0][5]],
                               [self.grasps_out_p[i].rotate_p[0][6],self.grasps_out_p[i].rotate_p[0][7],self.grasps_out_p[i].rotate_p[0][8]]
                              ])
            hdict['score']  = self.grasps_out_p[i].score
            self.grasps_hand.append(hdict)
            #print('pos:', hdict['pos'])
            #print('orient:', hdict['orient'])
            #print('rotate:', hdict['rotate'])
            #print('score:', hdict['score'])

        return self.grasps_hand
        
    def detectGraspsInCloud(self,config_filename, points):
        points = points.astype(np.float32) # flatten(order='C')
        row,col = points.shape
        size = row * col
        num_points = row
        points_c = (ctypes.c_float * size)(0)
        ctypes.memmove(points_c, points.ctypes.data, points.nbytes)

        camera_index = (ctypes.c_int32 * num_points)(1)
        #pdb.set_trace()
        
        num_view_points = 1
        num_v = 3*num_view_points
        view_points = (ctypes.c_float * num_v)(0.0,0.0,0.0)
        
        self.hand_num = self.lib.detectGraspsInCloud(config_filename,points_c,
                                                     camera_index,view_points,
                                                     num_points,num_view_points,
                                                     ctypes.byref(self.grasps_out_p))
        
        self.grasps_hand = []
        for i in range(self.hand_num):
            print('\ngrasp ',i)

            hdict = {}
            hdict['pos']    = np.array([self.grasps_out_p[i].pos_p[0][0],self.grasps_out_p[i].pos_p[0][1],self.grasps_out_p[i].pos_p[0][2]])
            hdict['orient'] = np.array([self.grasps_out_p[i].orient_p[0][0],self.grasps_out_p[i].orient_p[0][1],
                               self.grasps_out_p[i].orient_p[0][2],self.grasps_out_p[i].orient_p[0][3]])
            hdict['rotate'] = np.array([[self.grasps_out_p[i].rotate_p[0][0],self.grasps_out_p[i].rotate_p[0][1],self.grasps_out_p[i].rotate_p[0][2]],
                               [self.grasps_out_p[i].rotate_p[0][3],self.grasps_out_p[i].rotate_p[0][4],self.grasps_out_p[i].rotate_p[0][5]],
                               [self.grasps_out_p[i].rotate_p[0][6],self.grasps_out_p[i].rotate_p[0][7],self.grasps_out_p[i].rotate_p[0][8]]
                              ])
            hdict['score']  = self.grasps_out_p[i].score
            self.grasps_hand.append(hdict)
            #print('pos:', hdict['pos'])
            #print('orient:', hdict['orient'])
            #print('rotate:', hdict['rotate'])
            #print('score:', hdict['score'])

        return self.grasps_hand
    

    def get_grasp_hand(self):
        
        self.hand_num
        self.grasps_out_p


if __name__ == '__main__':
    so_path = '/root/gpd/build/'
    sys.path.append(so_path)
    #pdb.set_trace()
    os.chdir(so_path)

    if False:
        # 加载SO文件
        lib = ctypes.CDLL('/root/gpd/python_script/libgpd_c.so')
        # lib = ctypes.cdll.LoadLibrary('./libtest.so')

        '''
        extern "C" int detectGraspsInFile(char *config_filename, char *pcd_filename,
                                        char *normals_filename, float *view_points,
                                        int num_view_points,
                                        struct Grasp **grasps_out) {
        '''
        # 指定函数参数和返回值类型
        lib.detectGraspsInFile.argtypes = [ctypes.c_char_p, ctypes.c_char_p,
                                        ctypes.c_char_p, ctypes.POINTER(ctypes.c_float),
                                        ctypes.c_int32,
                                        ctypes.POINTER(ctypes.POINTER(StructGrasp))
                                        ]
        lib.detectGraspsInFile.restype = ctypes.c_int

        config_filename = b'../cfg/eigen_params.cfg'
        pcd_filename = b'/root/autodl-fs/31.pcd'
        normals_filename = b''
        num_view_points = 1
        # this is 2d array
        #view_points = (ctypes.c_float * 3 * num_view_points)()
        # this is 1d array
        size = 3 * num_view_points
        view_points = (ctypes.c_float * size)(0.0,0.0,0.0)
        #pdb.set_trace()
        # ctypes.POINTER 指针类型，用来给argtypes和restype指定参数为指针类型
        # ctypes.pointer 指针，pointer可以将实例对象转化为指针
        #grasps_out_p = ctypes.POINTER(StructGrasp())
        grasps_out_p = ctypes.pointer(StructGrasp())
        hand_num = lib.detectGraspsInFile(config_filename,pcd_filename,normals_filename,
                                        view_points,num_view_points,
                                        ctypes.byref(grasps_out_p))

        print('hand num: ', hand_num)

        #pdb.set_trace()
        '''
        extern "C" int detectGraspsInCloud(char *config_filename, float *points,
                                        int *camera_index, float *view_points,
                                        int size, int num_view_points,
                                        struct Grasp **grasps_out) {
        '''
    if True:
        gpd = CGPD()
        
        config_filename = b'../cfg/eigen_params.cfg'
        pcd_filename = b'/root/autodl-fs/1.pcd'
        normals_filename = b''
        #gpd.detectGraspsInFile(config_filename, pcd_filename, normals_filename)
        #pdb.set_trace()
        points = []
        with open(pcd_filename) as pf:
            lines = pf.readlines()
            '''
            # .PCD v.7 - Point Cloud Data file format
            VERSION .7
            FIELDS x y z rgb
            SIZE 4 4 4 4
            TYPE F F F I
            COUNT 1 1 1 1
            WIDTH 4467
            HEIGHT 1
            POINTS 4467
            DATA ascii
            '''
            for line in lines[12:]:
                dat = line.strip('\n').split(' ')
                if len(dat) < 3:
                    continue
                points.append([float(dat[0]),float(dat[1]),float(dat[2])])
                #pdb.set_trace()
        points = np.asarray(points)
        gpd.detectGraspsInCloud(config_filename, points)
        
