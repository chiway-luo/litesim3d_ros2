// Real renderer + native Qt input regression; requires a desktop graphics session.
#define LITESIM_VIEWER_TEST
#include "../src/viewer.cpp"
#include <filesystem>

int main(int argc,char **argv) {
  if(argc!=3&&argc!=4){std::cerr<<"usage: viewer_camera_test scene.json output-directory [embedded|pbr]\n";return 2;}
  rclcpp::init(0,nullptr);QApplication app(argc,argv);
  int result=0;
  try {
    const std::string materialMode=argc==4?argv[3]:"embedded";
    require(materialMode=="embedded"||materialMode=="pbr","unknown material expectation");
    std::filesystem::create_directories(argv[2]);
    rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("scene_description",argv[1])});
    auto node=std::make_shared<rclcpp::Node>("litesim_viewer_camera_test",options);
    Viewer viewer(node);viewer.show();app.processEvents();
    for(int i=0;i<6;++i)viewer.renderForTest();app.processEvents();
    const auto models=viewer.modelPoses();
    auto capture=[&](const char *name){viewer.renderForTest();app.processEvents();auto img=viewer.grab().toImage();
      require(img.save(QString::fromStdString((std::filesystem::path(argv[2])/name).string())),"cannot save test screenshot");return img;};
    auto original=capture("initial.png");
    // OBJ/DAE preserve the separate blue top. The explicit SDF PBR fixture
    // overrides every sub-material with the checker albedo, including the top.
    int gold=0,blue=0;
    for(int y=original.height()/3;y<original.height()*4/5;++y)
      for(int x=original.width()/2;x<original.width();++x){auto p=original.pixelColor(x,y);
        gold+=p.red()>p.green()*1.35&&p.green()>p.blue()*2&&p.red()>45;
        blue+=p.blue()>p.red()*1.8&&p.green()>p.red()*1.8&&p.blue()>80;}
    require(gold>500,"missing checker texture");
    require(materialMode=="pbr"?blue<500:blue>500,
      materialMode=="pbr"?"PBR override left the embedded blue top material":"missing independent top material");
    auto drag=[&](Qt::MouseButton button,Qt::KeyboardModifiers modifiers){
      QMouseEvent press(QEvent::MouseButtonPress,{500,350},button,button,modifiers);QApplication::sendEvent(&viewer,&press);
      QMouseEvent move(QEvent::MouseMove,{550,380},Qt::NoButton,button,modifiers);QApplication::sendEvent(&viewer,&move);
      QMouseEvent release(QEvent::MouseButtonRelease,{550,380},button,Qt::NoButton,modifiers);QApplication::sendEvent(&viewer,&release);};
    for(auto button:{Qt::MiddleButton,Qt::RightButton}){
      const auto locked=viewer.cameraPose();drag(button,Qt::NoModifier);
      require(viewer.cameraPose()==locked,"pan without Shift moved camera");
    }
    auto before=viewer.cameraPose();drag(Qt::LeftButton,Qt::NoModifier);
    require(viewer.cameraPose().Rot()!=before.Rot(),"left drag without Shift did not rotate camera");capture("orbit.png");
    before=viewer.cameraPose();drag(Qt::LeftButton,Qt::ShiftModifier);
    require(viewer.cameraPose().Pos()!=before.Pos()&&viewer.cameraPose().Rot()==before.Rot(),
      "Shift + left drag must translate camera without rotating");capture("pan.png");
    before=viewer.cameraPose();drag(Qt::MiddleButton,Qt::ShiftModifier);
    require(viewer.cameraPose()!=before,"Shift + middle drag did not pan camera");
    before=viewer.cameraPose();drag(Qt::RightButton,Qt::ShiftModifier);
    require(viewer.cameraPose()!=before,"Shift + right drag did not pan camera");
    QMouseEvent press(QEvent::MouseButtonPress,{500,350},Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(&viewer,&press);
    before=viewer.cameraPose();
    QMouseEvent unshifted(QEvent::MouseMove,{550,380},Qt::NoButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(&viewer,&unshifted);
    require(viewer.cameraPose().Rot()!=before.Rot(),"left drag did not rotate before pressing Shift");
    before=viewer.cameraPose();
    QMouseEvent shifted(QEvent::MouseMove,{560,385},Qt::NoButton,Qt::LeftButton,Qt::ShiftModifier);
    QApplication::sendEvent(&viewer,&shifted);
    require(viewer.cameraPose().Pos()!=before.Pos()&&viewer.cameraPose().Rot()==before.Rot(),
      "pressing Shift during left drag did not switch to pan");
    before=viewer.cameraPose();
    QMouseEvent resumed(QEvent::MouseMove,{570,390},Qt::NoButton,Qt::LeftButton,Qt::NoModifier);
    QApplication::sendEvent(&viewer,&resumed);
    require(viewer.cameraPose().Rot()!=before.Rot(),"releasing Shift during left drag did not resume rotation");
    QMouseEvent release(QEvent::MouseButtonRelease,{570,390},Qt::LeftButton,Qt::NoButton,Qt::NoModifier);
    QApplication::sendEvent(&viewer,&release);
    before=viewer.cameraPose();
    QWheelEvent wheel({550,350},{550,350},{0,0},{0,120},Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
    QApplication::sendEvent(&viewer,&wheel);
    require(viewer.cameraPose()!=before,"zoom input did not move camera");capture("zoom.png");
    viewer.resize(800,520);capture("resized.png");
    require(models==viewer.modelPoses(),"camera input changed scene models");
    std::cout<<J({{"status","PASS"},{"gold_pixels",gold},{"blue_pixels",blue},{"material_mode",materialMode},
      {"camera_operations",{"orbit","pan","zoom","resize"}},{"pan_requires_shift",true},
      {"pan_gesture","Shift + left drag"},
      {"model_poses_unchanged",true}}).dump(2)<<'\n';
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
  rclcpp::shutdown();return result;
}
