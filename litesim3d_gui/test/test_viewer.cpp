// Real renderer + native Qt input regression; requires a desktop graphics session.
#define LITESIM_VIEWER_TEST
#include "../src/viewer.cpp"
#include <filesystem>

int main(int argc,char **argv) {
  if(argc!=3){std::cerr<<"usage: viewer_camera_test scene.json output-directory\n";return 2;}
  rclcpp::init(0,nullptr);QApplication app(argc,argv);
  int result=0;
  try {
    std::filesystem::create_directories(argv[2]);
    rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("scene_description",argv[1])});
    auto node=std::make_shared<rclcpp::Node>("litesim_viewer_camera_test",options);
    Viewer viewer(node);viewer.show();app.processEvents();
    for(int i=0;i<6;++i)viewer.renderForTest();app.processEvents();
    const auto models=viewer.modelPoses();
    auto capture=[&](const char *name){viewer.renderForTest();app.processEvents();auto img=viewer.grab().toImage();
      require(img.save(QString::fromStdString((std::filesystem::path(argv[2])/name).string())),"cannot save test screenshot");return img;};
    auto original=capture("initial.png");
    // This detects the regression where normal-less OBJ imports turned white,
    // as well as accidental replacement of its separate blue top material.
    int gold=0,blue=0;
    for(int y=original.height()/3;y<original.height()*4/5;++y)
      for(int x=original.width()/2;x<original.width();++x){auto p=original.pixelColor(x,y);
        gold+=p.red()>p.green()*1.35&&p.green()>p.blue()*2&&p.red()>45;
        blue+=p.blue()>p.red()*1.8&&p.green()>p.red()*1.8&&p.blue()>80;}
    require(gold>500&&blue>500,"missing checker texture or independent top material");
    auto drag=[&](Qt::MouseButton button){
      QMouseEvent press(QEvent::MouseButtonPress,{500,350},button,button,Qt::NoModifier);QApplication::sendEvent(&viewer,&press);
      QMouseEvent move(QEvent::MouseMove,{550,380},Qt::NoButton,button,Qt::NoModifier);QApplication::sendEvent(&viewer,&move);
      QMouseEvent release(QEvent::MouseButtonRelease,{550,380},button,Qt::NoButton,Qt::NoModifier);QApplication::sendEvent(&viewer,&release);};
    auto before=viewer.cameraPose();drag(Qt::LeftButton);
    require(viewer.cameraPose()!=before,"orbit input did not move camera");capture("orbit.png");
    before=viewer.cameraPose();drag(Qt::MiddleButton);
    require(viewer.cameraPose()!=before,"pan input did not move camera");capture("pan.png");
    before=viewer.cameraPose();
    QWheelEvent wheel({550,350},{550,350},{0,0},{0,120},Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
    QApplication::sendEvent(&viewer,&wheel);
    require(viewer.cameraPose()!=before,"zoom input did not move camera");capture("zoom.png");
    viewer.resize(800,520);capture("resized.png");
    require(models==viewer.modelPoses(),"camera input changed scene models");
    std::cout<<J({{"status","PASS"},{"gold_pixels",gold},{"blue_pixels",blue},
      {"camera_operations",{"orbit","pan","zoom","resize"}},{"model_poses_unchanged",true}}).dump(2)<<'\n';
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
  rclcpp::shutdown();return result;
}
