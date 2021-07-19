#!/usr/bin/env python3
import argparse
import rospy
import torch
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openpifpaf import datasets
from openpifpaf import decoder, network, visualizer, show, logger, Predictor
from openpifpaf.predict import main, out_name
from monoloco.predict import get_torch_checkpoints_dir,download_checkpoints, factory_from_args, factory_outputs
from monoloco.visuals import Printer
from monoloco.network import net, Loco,preprocess_pifpaf, load_calibration
from monoloco.run import cli
import PIL
from PIL import Image as PIL_Image
import numpy as np
# from darknet_ros_msgs.msg import Coordinates
import time
import logging
import json
LOG = logging.getLogger(__name__)
def new_factory_from_args(args):

    # # Data
    # if args.glob:
    #     args.images += glob.glob(args.glob)
    # if not args.images:
    #     raise Exception("no image files given")

    # if args.path_gt is None:
    #     args.show_all = True

    # Models
    dic_models = download_checkpoints(args)
    args.checkpoint = dic_models['keypoints']

    # logger.configure(args, LOG)  # logger first

    # Devices
    args.device = torch.device('cpu')
    args.pin_memory = False
    if torch.cuda.is_available():
        args.device = torch.device('cuda')
        args.pin_memory = True
    LOG.debug('neural network device: %s', args.device)

    # Add visualization defaults
    if not args.output_types:
        args.output_types = ['multi']

    args.figure_width = 10
    args.dpi_factor = 1.0

    args.z_max = 10
    args.show_all = True
    args.no_save = True
    args.batch_size = 1

    if args.long_edge is None:
        args.long_edge = 144
    # Make default pifpaf argument
    args.force_complete_pose = True
    LOG.info("Force complete pose is active")

    # Configure
    # decoder.configure(args)
    # network.Factory.configure(args)
    # Predictor.configure(args)
    # show.configure(args)
    #visualizer.configure(args)

    return args, dic_models
    
def new_cli(argString):
    # Create an argument parser
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    subparsers = parser.add_subparsers(help='Different parsers for main actions', dest='command')
    # create the parser for the predict command
    predict_parser = subparsers.add_parser("predict")
    prep_parser = subparsers.add_parser("prep")
    training_parser = subparsers.add_parser("train")
    eval_parser = subparsers.add_parser("eval")

        # Predict (2D pose and/or 3D location from images)
    predict_parser.add_argument('images', nargs='*', help='input images')
    predict_parser.add_argument('--glob', help='glob expression for input images (for many images)')
    predict_parser.add_argument('--checkpoint', help='pifpaf model')
    predict_parser.add_argument('-o', '--output-directory', help='Output directory')
    predict_parser.add_argument('--output_types', nargs='+', default= [],
                                    help='MonoLoco - what to output: json bird front or multi')
    predict_parser.add_argument('--json-output', default=None, nargs='?', const=True,
                                    help='OpenpifPaf - whether to output a json file,'
                                        'with the option to specify the output path or directory')
    predict_parser.add_argument('--no_save', help='to show images', action='store_true')
    predict_parser.add_argument('--hide_distance', help='to not show the absolute distance of people from the camera',
                                    default=False, action='store_true')
    predict_parser.add_argument('--dpi', help='image resolution', type=int, default=100)
    predict_parser.add_argument('--long-edge', default=None, type=int,
                                    help='rescale the long side of the image (aspect ratio maintained)')
    predict_parser.add_argument('--white-overlay',
                                    nargs='?', default=False, const=0.8, type=float,
                                    help='increase contrast to annotations by making image whiter')
    predict_parser.add_argument('--font-size', default=0, type=int, help='annotation font size')
    predict_parser.add_argument('--monocolor-connections', default=False, action='store_true',
                                    help='use a single color per instance')
    predict_parser.add_argument('--instance-threshold', type=float, default=None, help='threshold for entire instance')
    predict_parser.add_argument('--seed-threshold', type=float, default=0.5, help='threshold for single seed')
    predict_parser.add_argument('--disable-cuda', action='store_true', help='disable CUDA')
    predict_parser.add_argument('--precise-rescaling', dest='fast_rescaling', default=True, action='store_false',
                                    help='use more exact image rescaling (requires scipy)')
    predict_parser.add_argument('--decoder-workers', default=None, type=int,
                                    help='number of workers for pose decoding, 0 for windows')

    # decoder.cli(parser)
    # logger.cli(parser)
    # network.Factory.cli(parser)
    # show.cli(parser)
    # visualizer.cli(parser)

        # Monoloco
    predict_parser.add_argument('--activities', nargs='+', choices=['raise_hand', 'social_distance'],
                                    help='Choose activities to show: social_distance, raise_hand', default=[])
    predict_parser.add_argument('--mode', help='keypoints, mono, stereo', default='mono')
    predict_parser.add_argument('--model', help='path of MonoLoco/MonStereo model to load')
    predict_parser.add_argument('--net', help='only to select older MonoLoco model, otherwise use --mode')
    predict_parser.add_argument('--path_gt', help='path of json file with gt 3d localization')
                                    #default='data/arrays/names-kitti-200615-1022.json')
    predict_parser.add_argument('--z_max', type=int, help='maximum meters distance for predictions', default=100)
    predict_parser.add_argument('--n_dropout', type=int, help='Epistemic uncertainty evaluation', default=0)
    predict_parser.add_argument('--dropout', type=float, help='dropout parameter', default=0.2)
    predict_parser.add_argument('--show_all', help='only predict ground-truth matches or all', action='store_true')
    predict_parser.add_argument('--webcam', help='monstereo streaming', action='store_true')
    predict_parser.add_argument('--camera', help='device to use for webcam streaming', type=int, default=0)
    predict_parser.add_argument('--calibration', help='type of calibration camera, either custom, nuscenes, or kitti',
                                    type=str, default='custom')
    predict_parser.add_argument('--focal_length',
                                    help='foe a custom camera: focal length in mm for a sensor of 7.2x5.4 mm. (nuScenes)',
                                    type=float, default=5.7)

        # Social distancing and social interactions
    predict_parser.add_argument('--threshold_prob', type=float, help='concordance for samples', default=0.25)
    predict_parser.add_argument('--threshold_dist', type=float, help='min distance of people', default=2.5)
    predict_parser.add_argument('--radii', type=tuple, help='o-space radii', default=(0.3, 0.5, 1))
        # ArgumentParser parses arguments through the parse_args() method. This will inspect the command line, 
        # convert each argument to the appropriate type and then invoke the appropriate action
    args = parser.parse_args(argString.split(" "))

    return args
        
class Visualizer:
    def __init__(self, kk, args):
        self.kk = kk
        self.args = args

    def __call__(self, first_image, fig_width=1.0, **kwargs):
        if 'figsize' not in kwargs:
            kwargs['figsize'] = (fig_width, fig_width *
                                 first_image.size[0] / first_image.size[1])

        printer = Printer(first_image, output_path="",
                          kk=self.kk, args=self.args)

        figures, axes = printer.factory_axes(None)

        for fig in figures:
            fig.show()

        while True:
            image, dic_out, pifpaf_outs = yield

            # Clears previous annotations between frames
            axes[0].patches = []
            axes[0].lines = []
            axes[0].texts = []
            if len(axes) > 1:
                axes[1].patches = []
                axes[1].lines = [axes[1].lines[0], axes[1].lines[1]]
                axes[1].texts = []

            if dic_out and dic_out['dds_pred']:
                printer._process_results(dic_out)
                printer.draw(figures, axes, image, dic_out, pifpaf_outs['left'])
                mypause(0.01)

class DetectorManager():
    def __init__(self):
        #Load image parameter and confidence threshold
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')

        # # Load publisher topics
        # self.detected_objects_topic = rospy.get_param('~detected_objects_topic')

        # Load other parameters
        self.argString = rospy.get_param('~monoloco_args', 'predict docs/frame0032.jpg --activities social_distance --output_types front bird')

        # Load net

        rospy.loginfo('People detection starting')
        # Define subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size = 1, buff_size = 2**24)

        # Define publishers

        # Spin
        rospy.spin()


    def imageCb(self,data):
        args = new_cli(self.argString)
        # Convert the image to OpenCV
        #import pdb;pdb.set_trace()
        cnt = 0
        assert args.mode in ('mono')
        args, dic_models = new_factory_from_args(args)
        # Load Models
        net = Loco(model=dic_models[args.mode], mode=args.mode, device=args.device, n_dropout=args.n_dropout, p_dropout=args.dropout)
        # for openpifpaf predictions
        predictor = Predictor(checkpoint=args.checkpoint)
        visualizer_mono = None

            #self.cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        try:
            bridge = CvBridge()
            self.cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
            height, width, _ = self.cv_image.shape
        except CvBridgeError as e:
            print(e)
            #height, width, _ = self.cv_image.shape
        pil_image = PIL_Image.fromarray(self.cv_image)

        data1 = datasets.PilImageList([pil_image], preprocess=predictor.preprocess)

        data_loader = torch.utils.data.DataLoader( data1, batch_size=1, shuffle=False,pin_memory=False, collate_fn=datasets.collate_images_anns_meta)
        pifpaf_outs = {}
        for (_, _, _) in data_loader:
            for idx, (preds, _, _) in enumerate(predictor.dataset(data1)):
                if idx == 0:
                    pifpaf_outs = { 
                        'pred': preds,
                        'left': [ann.json_data() for ann in preds],
                        'image': self.cv_image}


        kk = load_calibration(args.calibration, pil_image.size, focal_length=args.focal_length)
        boxes, keypoints = preprocess_pifpaf(pifpaf_outs['left'], (width, height))
        dic_out = net.forward(keypoints, kk)
        dic_out = net.post_process(dic_out, boxes, keypoints, kk)     

        if 'social_distance' in args.activities:
            dic_out = net.social_distance(dic_out, args)
        print(json.dumps(dic_out))
            # import pdb;pdb.set_trace()
            # if visualizer_mono is None:  # it is, at the beginning
            #     visualizer_mono = Visualizer(kk, args)(pil_image)  # create it with the first image
            #     visualizer_mono.send(None)



if __name__=="__main__":
    # Initialize node
    rospy.init_node("detector_manager_node")

    # Define detector object
    dm = DetectorManager()


    # def imageCb(self,data):
    #     args = new_cli(self.argString)
    #     # Convert the image to OpenCV
    #     #import pdb;pdb.set_trace()
    #     try:
    #         bridge = CvBridge()
    #         self.cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
    #         height, width, _ = self.cv_image.shape
    #     except CvBridgeError as e:
    #         print(e)

    #     pil_image = PIL_Image.fromarray(self.cv_image)

    #     cnt = 0
    #     assert args.mode in ('mono')
    #     args, dic_models = new_factory_from_args(args)
    #     # Load Models
    #     net = Loco(model=dic_models[args.mode], mode=args.mode, device=args.device, n_dropout=args.n_dropout, p_dropout=args.dropout)
    #     # for openpifpaf predictions
    #     predictor = Predictor(checkpoint=args.checkpoint)

    #     data = datasets.PilImageList([pil_image], preprocess=predictor.preprocess)

    #     data_loader = torch.utils.data.DataLoader(data, batch_size=1, shuffle=False,pin_memory=False, collate_fn=datasets.collate_images_anns_meta)
    #     pifpaf_outs = {}
    #     start = time.time()
    #     timing = []
    #     for (_, _, _) in data_loader:

    #         for idx, (preds, _, _) in enumerate(predictor.dataset(data)):

    #             if idx == 0:
    #                 pifpaf_outs = {
    #                     'pred': preds,
    #                     'left': [ann.json_data() for ann in preds],
    #                     'image': self.cv_image}

    #     kk = load_calibration(args.calibration, pil_image.size, focal_length=args.focal_length)
    #     # Preprocess pifpaf outputs and run monoloco
    #     boxes, keypoints = preprocess_pifpaf(pifpaf_outs['left'], (width, height))

    #     dic_out = net.forward(keypoints, kk)
    #     dic_out = net.post_process(dic_out, boxes, keypoints, kk)               

    #     if 'social_distance' in args.activities:
    #         dic_out = net.social_distance(dic_out, args)
    #     if 'raise_hand' in args.activities:
    #         dic_out = net.raising_hand(dic_out, keypoints)
    #         # Output
    #     factory_outputs(args, pifpaf_outs, dic_out, output_path, kk=kk)
    #         #print(f'Image {cnt}\n' + '-' * 120)
    #     cnt += 1
    #     #start = time.time()