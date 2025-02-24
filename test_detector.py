import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class FakeDetector(Node):
    def __init__(self):
        super().__init__('test_detector')
        
        # Define the waypoints to be used for object detection
        self.valid_waypoints = [
    (float(60.0), float(-20.0), float(2.0)),
    (float(60.03370294958586), float(-19.557055815297645), float(2.0)),
    (float(60.07493421784602), float(-19.008821080613416), float(2.0)),
    (float(60.12232464936096), float(-18.37064422329422), float(2.0)),
    (float(60.174017690180335), float(-17.664714054437354), float(2.0)),
    (float(60.22803454339737), float(-16.915665005217306), float(2.0)),
    (float(60.28268798568752), float(-16.14569966052659), float(2.0)),
    (float(60.3369618775323), float(-15.370194195391377), float(2.0)),
    (float(60.39073059591465), float(-14.594653523585293), float(2.0)),
    (float(60.44432087306632), float(-13.819100647000596), float(2.0)),
    (float(60.49799347843509), float(-13.04355336673325), float(2.0)),
    (float(60.55192939005792), float(-12.268024046119535), float(2.0)),
    (float(60.60625532310223), float(-11.492521580599714), float(2.0)),
    (float(60.66104367701337), float(-10.717051331710536), float(2.0)),
    (float(60.7162797737401), float(-9.941612560360227), float(2.0)),
    (float(60.77182379132137), float(-9.16619536280632), float(2.0)),
    (float(60.82737576245563), float(-8.390778143366333), float(2.0)),
    (float(60.8824081311468), float(-7.615323161444394), float(2.0)),
    (float(60.9360456088325), float(-6.839773548301309), float(2.0)),
    (float(60.98706374608446), float(-6.064065767917782), float(2.0)),
    (float(61.03421048622113), float(-5.28815667348681), float(2.0)),
    (float(61.076557118445635), float(-4.512041818437865), float(2.0)),
    (float(61.1135972031625), float(-3.7357538700453006), float(2.0)),
    (float(61.14531868346967), float(-2.9593499618931673), float(2.0)),
    (float(61.17240511602722), float(-2.1828919193067122), float(2.0)),
    (float(61.196395748702344), float(-1.4064221441512927), float(2.0)),
    (float(61.21946783323074), float(-0.6299527067458257), float(2.0)),
    (float(61.243766622734256), float(0.1465169372386299), float(2.0)),
    (float(61.270649312005844), float(0.922974602523027), float(2.0)),
    (float(61.30021426989697), float(1.6994001598504838), float(2.0)),
    (float(61.33113482396584), float(2.4757921001291834), float(2.0)),
    (float(61.36079532088479), float(3.2521712275629397), float(2.0)),
    (float(61.38588514184812), float(4.028538098267745), float(2.0)),
    (float(61.40320184780285), float(4.804820045654196), float(2.0)),
    (float(61.409965165075846), float(5.580835900007514), float(2.0)),
    (float(61.40358211682178), float(6.356216219312046), float(2.0)),
    (float(61.38144906552043), float(7.1302699440857396), float(2.0)),
    (float(61.34108518564608), float(7.901947458914947), float(2.0)),
    (float(61.28023796528578), float(8.669932096352568), float(2.0)),
    (float(61.19648196030175), float(9.432604038418503), float(2.0)),
    (float(61.086725929926615), float(10.187727545184316), float(2.0)),
    (float(60.94746743567521), float(10.932276659936178), float(2.0)),
    (float(60.77552001288859), float(11.662579928059131), float(2.0)),
    (float(60.56834295054432), float(12.374384675291367), float(2.0)),
    (float(60.324125253479), float(13.062892278743675), float(2.0)),
    (float(60.041943790216465), float(13.722983364626998), float(2.0)),
    (float(59.7216892404831), float(14.349375527293887), float(2.0)),
    (float(59.36376606684644), float(14.936557480425108), float(2.0)),
    (float(58.969083067262545), float(15.478848582220962), float(2.0)),
    (float(58.53945926926099), float(15.97093388333451), float(2.0)),
    (float(58.07790774572641), float(16.408651153644314), float(2.0)),
    (float(57.588357207307126), float(16.789438455482014), float(2.0)),
    (float(57.07513375201961), float(17.112387263507117), float(2.0)),
    (float(56.542697246943135), float(17.378434083861066), float(2.0)),
    (float(55.99551433126908), float(17.59070336841978), float(2.0)),
    (float(55.43770959653193), float(17.754361098923255), float(2.0)),
    (float(54.8726925625233), float(17.876123134599766), float(2.0)),
    (float(54.303097365133), float(17.96396451763576), float(2.0)),
    (float(53.730833737994544), float(18.02659406332532), float(2.0)),
    (float(53.15714367327746), float(18.072465792356525), float(2.0)),
    (float(52.58277192834066), float(18.10893021497759), float(2.0)),
    (float(52.0081654448295), float(18.14178220159374), float(2.0)),
    (float(51.43359127576696), float(18.175023199175484), float(2.0)),
    (float(50.85918940673582), float(18.210830663505476), float(2.0)),
    (float(50.285002273798455), float(18.249809447443113), float(2.0)),
    (float(49.71099872404011), float(18.29144390503643), float(2.0)),
    (float(49.137102678767405), float(18.334619699046016), float(2.0)),
    (float(48.56322935590288), float(18.37814331089612), float(2.0)),
    (float(47.98931969393743), float(18.421185961778974), float(2.0)),
    (float(47.41535598860355), float(18.463487396627897), float(2.0)),
    (float(46.84134871355491), float(18.5051843887195), float(2.0)),
    (float(46.267310955154244), float(18.546456544136163), float(2.0)),
    (float(45.69324683875311), float(18.587359404249582), float(2.0)),
    (float(45.119155822496396), float(18.627883100649342), float(2.0)),
    (float(44.54503756802296), float(18.66802099061897), float(2.0)),
    (float(43.97089216171298), float(18.70777229568921), float(2.0)),
    (float(43.39671972341603), float(18.747136669728206), float(2.0)),
    (float(42.82252073683776), float(18.786118999298196), float(2.0)),
    (float(42.248294279619586), float(18.824703931750264), float(2.0)),
    (float(41.67403482936788), float(18.862808576552197), float(2.0)),
    (float(41.09973486344097), float(18.900318348547444), float(2.0)),
    (float(40.52539522195002), float(18.937238294398412), float(2.0)),
    (float(39.9510321757989), float(18.973802170075942), float(2.0)),
    (float(39.37667396903271), float(19.01042537621106), float(2.0)),
    (float(38.80235272709979), float(19.047581925347913), float(2.0)),
    (float(38.22809802641859), float(19.085702630021842), float(2.0)),
    (float(37.65393212373601), float(19.125100612756796), float(2.0)),
    (float(37.079865568259265), float(19.165911311574746), float(2.0)),
    (float(36.50589524884708), float(19.20807447299012), float(2.0)),
    (float(35.93200650613289), float(19.251372264698148), float(2.0)),
    (float(35.35817307175603), float(19.29543262720108), float(2.0)),
    (float(34.784349652414676), float(19.339637593366206), float(2.0)),
    (float(34.210477015643846), float(19.383190743013984), float(2.0)),
    (float(33.63652054424165), float(19.425610427744687), float(2.0)),
    (float(33.06251150579192), float(19.467281549063046), float(2.0)),
    (float(32.48855081020156), float(19.509520119434455), float(2.0)),
    (float(31.91478888056008), float(19.554255054041278), float(2.0)),
    (float(31.341409268497955), float(19.603688075614627), float(2.0)),
    (float(30.768615471373778), float(19.65997604414588), float(2.0)),
    (float(30.196608522033785), float(19.72487433173228), float(2.0)),
    (float(29.625552482903004), float(19.799408923834562), float(2.0)),
    (float(29.055533473321702), float(19.883663666725624), float(2.0)),
    (float(28.48652278567897), float(19.976714201038703), float(2.0)),
    (float(27.918365914782044), float(20.076770613493863), float(2.0)),
    (float(27.35082283092197), float(20.18161304615205), float(2.0)),
    (float(26.783649485500064), float(20.2892155109148), float(2.0)),
    (float(26.216658925346565), float(20.39814426092198), float(2.0)),
    (float(25.649713152146433), float(20.507401577779092), float(2.0)),
    (float(25.0826749157859), float(20.615993875282584), float(2.0)),
    (float(24.51536308095092), float(20.72261446408811), float(2.0)),
    (float(23.947521555470303), float(20.825316851201933), float(2.0)),
    (float(23.378866528859362), float(20.92148321395507), float(2.0)),
    (float(22.80921247106744), float(21.00900767830899), float(2.0)),
    (float(22.2384697982925), float(21.087590225972235), float(2.0)),
    (float(21.666549349203706), float(21.157005889108405), float(2.0)),
    (float(21.09339323156746), float(21.2136183619441), float(2.0)),
    (float(20.51905422285199), float(21.250042537692934), float(2.0)),
    (float(19.943809178483207), float(21.257205745729152), float(2.0)),
    (float(19.36848812084645), float(21.225183440721594), float(2.0)),
    (float(18.79490754904691), float(21.143200233695097), float(2.0)),
    (float(18.226305531163234), float(21.000657004478853), float(2.0)),
    (float(17.667506746947765), float(20.789369839592837), float(2.0)),
    (float(17.12390478159068), float(20.505265526939183), float(2.0)),
    (float(16.602182979986537), float(20.147056393674575), float(2.0)),
    (float(16.111468791554216), float(19.714314875018317), float(2.0)),
    (float(15.66278621845413), float(19.207261253061006), float(2.0)),
    (float(15.269482228148263), float(18.62667564250296), float(2.0)),
    (float(14.94801299663959), float(17.975202582398197), float(2.0)),
    (float(14.716810441343114), float(17.262162242550403), float(2.0)),
    (float(14.592296770948451), float(16.50782169419108), float(2.0)),
    (float(14.584659326646943), float(15.74146162183024), float(2.0)),
    (float(14.695450639526825), float(14.995542863325682), float(2.0)),
    (float(14.91828057722887), float(14.300871968793217), float(2.0)),
    (float(15.241577112930827), float(13.684884790680371), float(2.0)),
    (float(15.648865432478487), float(13.17011380114127), float(2.0)),
    (float(16.115035944385454), float(12.767779542482458), float(2.0)),
    (float(16.60851262079086), float(12.472568336175755), float(2.0)),
    (float(17.097240567556582), float(12.267621547973249), float(2.0)),
    (float(17.55207851965679), float(12.133409164496697), float(2.0)),
    (float(17.94982484355569), float(12.052034117281437), float(2.0)),
    (float(18.27492285327753), float(12.008262952324003), float(2.0)),
]


        # Initialize the publisher
        self.publisher_ = self.create_publisher(String, 'detected_object', 10)
        # Create a timer to publish detections at intervals
        self.timer = self.create_timer(2, self.publish_fake_detection) #2
        self.i = 0

    def publish_fake_detection(self):
        # Randomly select a waypoint from the valid waypoints
        selected_waypoint = random.choice(self.valid_waypoints)
        latitude, longitude, _ = selected_waypoint

        # Randomly choose an object ID
        object_ids = [1,2,3,4,5,6,7,8]
        object_id = object_ids[self.i]
        self.i += 1
        if self.i == 8: self.i = 0

        # Construct the detection message
        detection_message = f"Detected Object ID: {object_id}, Location: {latitude}, {longitude}"
        
        # Publish the message
        msg = String()
        msg.data = detection_message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {detection_message}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
