from matplotlib import pyplot as plt
import numpy as np
import cv2
from os.path import join


def save_flow(flow, img, name, prefix):
    hsv_mask = np.zeros_like(img) 
    hsv_mask = np.expand_dims(hsv_mask, axis=2)
    hsv_mask = np.dstack((hsv_mask, hsv_mask, hsv_mask))
    # Make image saturation to a maximum value 
    hsv_mask[..., 1] = 255
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1]) 
    # Set image hue value according to the angle of optical flow 

    hsv_mask[..., 0] = ang * 180 / np.pi / 2
    # Set value as per the normalized magnitude of optical flow 
    hsv_mask[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX) 
    # Convert to rgb 
    rgb_representation = cv2.cvtColor(hsv_mask, cv2.COLOR_HSV2BGR)

    path = '/home/joanreyero/ros_workspaces/pyx4_ws/src/pyx4_avoidance/src/analytics/flows'
    
    save = join(path, prefix + '-' + str(name) + '.png')
    cv2.imwrite(save, rgb_representation)



def draw_flow(flow, img, step=5, filter_img=None, new=True, save=False):
    """

    :param flow:
    :param img:
    :param step:
    :param filter_img: optional - if using matched filters we can overlay the image filter defined by this variable
    :return:
    """

    # if empty_im_flag:
    #     img = np.ones((flow.shape[0], flow.shape[1], 3))
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1)
    x = x.astype(np.int64)
    y = y.astype(np.int64)

    # def f(vec):
    #     u = vec[:2]
    #     v = vec[2:]
    #     return (np.dot(u, v)/np.dot(v, v))*v
    
    # # print flow[x, y]
    # flow = np.apply_along_axis(f, 2, np.concatenate((flow, filter_img), axis=2))
    fx, fy = flow[y, x].T * 20
    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)

    try:
        cv2.polylines(img, lines, 0, (0, 255, 0))

        # if filter_img is not None:
        #     # print ('lines 2 {}'.format(fx))
        #     fx, fy = filter_img[y, x].T * 20
        #     lines_filt = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
        #     lines_filt = np.int32(lines_filt + 0.5)
        #     # print ('lines 2 {}'.format(np.shape(lines)))
        #     cv2.polylines(img, lines_filt, 0, (255, 0, 0))

    except Exception as e:
        raise('exception {}, check the dimensions of img and try copying (.copy()) it from its source before it into this (draw_flow) function'.format(e))

    # print(flow.shape)
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(img, (x1, y1), 1, (0, 255, 0), -1)
    return img


def draw_hsv(flow):
    h, w = flow.shape[:2]
    fx, fy = flow[:,:,0], flow[:,:,1]
    ang = np.arctan2(fy, fx) + np.pi
    v = np.sqrt(fx*fx+fy*fy)
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[...,0] = ang*(180/np.pi/2)
    hsv[...,1] = 255
    hsv[...,2] = np.minimum(v*4, 255)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return bgr


def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:,np.newaxis]
    res = cv2.remap(img, flow, None, cv2.INTER_LINEAR)
    return res


def plot_flow(left_filter, right_filter, ax_left=None, ax_right=None, step_size=40, scale=None, do_plot=True, cam=None):

    if not ax_left:
        fig1, (ax_left, ax_right) = plt.subplots(2, 1)

    ax_left.set_title('Left Filter values')
    # Q = ax1.quiver(X, Y, left_filter[:,:,0], left_filter[:,:,1], units='width')

    X, Y, U, V = get_plot_flow_components(flow_field_in=left_filter)
    # Q1 = ax1.quiver(X[::step_size, ::step_size], Y[::step_size, ::step_size], U[::step_size, ::step_size],
    #                V[::step_size, ::step_size], pivot='mid')
    Q1 = ax_left.quiver(U[::step_size, ::step_size], V[::step_size, ::step_size], pivot='mid', scale=scale)

    X, Y, U, V = get_plot_flow_components(flow_field_in=right_filter)
    ax_right.set_title('Right Filter values')
    Q2 = ax_right.quiver(U[::step_size, ::step_size], V[::step_size, ::step_size], pivot='mid', scale=scale)

    if do_plot:
        plt.show()

def plot_flow_single(flow, ax_left=None, step_size=40, scale=None, do_plot=True, cam_in=None, title=None, fontsize=10):

    if not ax_left:
        fig1, (ax_left, ax_right) = plt.subplots(2, 1)

    if cam_in:
        X, Y, U, V = get_plot_flow_components_degrees(flow_field_in=flow, cam_in=cam_in)
    else:
        X, Y, U, V = get_plot_flow_components(flow_field_in=flow)

    Q1 = ax_left.quiver(X[::step_size, ::step_size], Y[::step_size, ::step_size], U[::step_size, ::step_size], V[::step_size, ::step_size], pivot='mid', scale=scale)

    if title:
        ax_left.set_title(title, fontsize=fontsize)


def get_plot_flow_components(flow_field_in):
    X, Y = np.meshgrid(np.arange(0, flow_field_in.shape[1], 1.0), np.arange(0, flow_field_in.shape[0], 1.0))
    U = flow_field_in[:, :, 0]
    V = flow_field_in[:, :, 1]
    return X, Y, U, V

def get_pixel_coordinates(cam_in, ):
    half_width = cam_in.fov_hor_degs / 2.0
    half_height = cam_in.fov_vert_degs / 2.0

    vert_degs = np.linspace(-half_height, half_height, cam_in.cam_h)
    hor_degs = np.linspace(-half_width, half_width, cam_in.cam_w)

    X, Y = np.meshgrid(hor_degs, vert_degs)

    return X, Y

def get_plot_flow_components_degrees(flow_field_in, cam_in):

    midpoint_w = cam_in.cam_w/2.0
    midpoint_h = cam_in.cam_h/2.0

    half_width = cam_in.fov_hor_degs/2.0
    half_height = cam_in.fov_vert_degs/2.0

    vert_degs = np.linspace(-half_height, half_height, flow_field_in.shape[0])
    hor_degs = np.linspace(-half_width, half_width, flow_field_in.shape[1])

    X, Y = np.meshgrid(hor_degs, vert_degs)
    U = flow_field_in[:, :, 0]
    V = flow_field_in[:, :, 1]
    return X, Y, U, V