/*
@author Lucas Grazziotim <lucasgrazziotim@gmail.com>
*/

#include <multi_uav/Formation.h>

namespace multi_uav{

Formation::Formation(std::vector<multi_uav::Drone*> drones)
{
  this->numDrones = drones.size();
  this->xmoved = 0.0;
  this->ymoved = 0.0;
  this->zmoved = 0.0;
  this->moveX = 0.0;
  this->moveY = 0.0;

  this->droneDistance = 1.0;
  this->apertureAngle = 90.0;    // degrees
  this->yawAngle = 0.0;          // degrees
  this->height = 2.0;
  this->currentFormation = 1;

  this->waitEnable = false;

  this->drones = drones;

  for(int i=0; i<this->numDrones; i++){
    position p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = TAKEOFFHEIGHT;
    p.theta = 0.0;
    this->posDrones.push_back(p);
  }
}

Formation::~Formation()
{

}

void Formation::setMoveOffset(double offset){
  this->MOVEJM = offset;
}

void Formation::setRotateOffset(double offset){
  this->ROTATEJM = offset;
}

void Formation::setDistanceOffset(double offset){
  this->DISTANCEJM = offset;
}

void Formation::setApertureOffset(double offset){
  this->APERTUREJM = offset;
}

void Formation::setUpDownOffset(double offset){
  this->UPDOWNJM = offset;
}

std::vector<multi_uav::Drone*> Formation::getDrones()
{
  return this->drones;
}

void Formation::setWaitEnable(bool enable)
{
  this->waitEnable = enable;
}


void Formation::droneLocalControl(int droneNumber ){

  int droneFound = -1;

  // get drone
  for (int i=0; i<this->drones.size(); i++) {
    if(this->drones[i]->parameters.id == droneNumber){
      droneFound = i;
      break;
    }
  }

  if(droneFound != -1){
    this->drones[droneFound]->configureToUseLocalCoordinates();
    this->drones[droneFound]->forceModeOffboard();
    this->drones[droneFound]->arm();
    ros::Rate rate(10);
    while(ros::ok()){
      this->drones[droneFound]->goToLocalPosition(this->posDrones[droneFound].x, this->posDrones[droneFound].y, this->posDrones[droneFound].z, this->posDrones[droneFound].theta, this->waitEnable);
      rate.sleep();
    }
    this->drones[droneFound]->~Drone();
  }
}

int Formation::getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();  // read character (blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  usleep(100000);
  return c;
}


////////////////////////////////// Print Information //////////////////////////////////////////
void Formation::printCoordinates()
{
  int centralDrone = (numDrones-1)/2;
  std::cout << std::endl << "****************  Destination Coordinates  ***************" << std::endl;
  for(int i=0; i<numDrones; i++){
      std::cout << "i= " << i << " ";
      std::cout << "X= " << this->posDrones[i].x << "  ";
      std::cout << "Y= " << this->posDrones[i].y << "  ";
      std::cout << "Z= " << this->posDrones[i].z << "  ";
      std::cout << "theta= " << this->posDrones[i].theta << std::endl;
    }
  std::cout << "**********************************************************" << std::endl;
}

void Formation::printCurrentFormationInfo()
{
  double radius = this->droneDistance;

  std::cout << std::endl << "Formation: ";
  switch (this->currentFormation)
  {
    case 1:
      std::cout << "LINE" << std::endl;
      break;
    case 2:
      std::cout << "ARROW" << std::endl;
      break;
    case 3:
      std::cout << "CROSS" << std::endl;
      break;
    case 4:
      std::cout << "CIRCLE - radius: " << radius << std::endl;
      break;
  }
  std::cout << "Drone distance: " << this->droneDistance << std::endl;
  std::cout << "Aperture angle: " << this->apertureAngle << std::endl;
  std::cout << "Yaw angle: " << this->yawAngle << std::endl;
  std::cout << "X moved: " << this->xmoved;
  std::cout << "  Y moved: " << this->ymoved;
  std::cout << "  Z moved: " << this->zmoved;
  std::cout << "  Height: " << this->height << std::endl;
}

void Formation::printJoystickControls()
{
  std::cout << std::endl << "********************  Joystick Control  ********************" << std::endl;
  std::cout << "Press the following keys to control the UAV formation: " << std::endl;
  std::cout << "w s d a     =>  moves the formation forward / backward / right / left" << std::endl;
  std::cout << "f / g       =>  moves the formation up / down" << std::endl;
  std::cout << "q / e       =>  rotates the formation counter-clk/clk wise" << std::endl;
  std::cout << "1 ... 9     =>  changes current formation " << std::endl;
  std::cout << "o / p       =>  increase/decrease distance / radius between drones " << std::endl;
  std::cout << "k / l       =>  increase/decrease aperture angle" << std::endl;
  std::cout << "esc         =>  closes joystick mode " << std::endl;
  std::cout << "************************************************************" << std::endl;
}

void Formation::printJoystickCommands(int command)
{
  switch (command)
  {
    //  Move
    case 119:
      std::cout << " Move Forwards: " << MOVEJM << "m" << std::endl;
      break;
    case 115:
      std::cout << " Move Backwards: " << MOVEJM << "m" << std::endl;
      break;
    case 100:
      std::cout << " Move Rigth: " << MOVEJM << "m" << std::endl;
      break;
    case 97:
      std::cout << " Move Left: " << MOVEJM << "m" << std::endl;
      break;

    //  Rotate
    case 113:
      std::cout << " Rotate Counter Clock Wise: " << ROTATEJM << "degrees" << std::endl;
      break;
    case 101:
      std::cout << " Rotate Clock Wise: " << ROTATEJM << "degrees" << std::endl;
      break;

    //  Distance
    case 111:
      std::cout << " Distance Increased: " << DISTANCEJM << "m" << std::endl;
      break;
    case 112:
      std::cout << " Distance Decreased: " << DISTANCEJM << "m" << std::endl;
      break;
    case 107:
      std::cout << " Aperture Angle Increased: " << APERTUREJM << "degrees" << std::endl;
      break;
    case 108:
      std::cout << " Aperture Angle Decreased: " << APERTUREJM << "degrees" << std::endl;
      break;

    // Formation
    case 49:
      std::cout << " Formation changed to: LINE " << std::endl;
      break;

    case 50:
      std::cout << " Formation changed to: ARROW " << std::endl;
      break;

    case 51:
      std::cout << " Formation changed to: CROSS " << std::endl;
      break;

    case 52:
      std::cout << " Formation changed to: CIRCLE " << std::endl;
      break;

    case 53:
      //std::cout << " Formation changed to: LINE " << std::endl;
      break;

    case 54:
      //std::cout << " Formation changed to: LINE " << std::endl;
      break;

    case 55:
      //std::cout << " Formation changed to: LINE " << std::endl;
      break;

    default:
      break;

  }

}


/////////////////////////////////////// Formations /////////////////////////////////////////////
void Formation::circle(double droneDistance,                       double yawAngle, double height)
{
  this->droneDistance = droneDistance;
  this->yawAngle = yawAngle;
  this->height = height;
  this->currentFormation = 4;
  this->zmoved = height - TAKEOFFHEIGHT;

  int centralDrone = (this->numDrones-1)/2, i, posIndex;
  double cx=drones[centralDrone]->parameters.position.local.x;
  double cy=drones[centralDrone]->parameters.position.local.y;
  double x0=0.0, y0=0.0;

  double radius = droneDistance;
  double deltaAngle = 360/(this->numDrones-1);
  deltaAngle = multi_uav::utils::Math::degreesToRadians(deltaAngle);

  //yawAngle is not a pointer, changes are not seen outside this function
  yawAngle = multi_uav::utils::Math::degreesToRadians(yawAngle);


  for(i=0, posIndex = centralDrone+1; posIndex<(this->numDrones-(this->numDrones%2)); posIndex+=2)  {
    x0 = radius*cos(i*deltaAngle);
    y0 = radius*sin(i*deltaAngle);

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - OFFSETORIGIN*(posIndex-centralDrone);
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle+i*deltaAngle);

    i++;
  }

  for(posIndex = this->numDrones-2+ (this->numDrones%2); posIndex>centralDrone; posIndex-=2)  {
    x0 = radius*cos(i*deltaAngle);
    y0 = radius*sin(i*deltaAngle);

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - OFFSETORIGIN*(posIndex-centralDrone);
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle+i*deltaAngle);

    i++;
  }



  for(posIndex = centralDrone-1; posIndex>0; posIndex-=2)  {
    x0 = radius*cos(i*deltaAngle);
    y0 = radius*sin(i*deltaAngle);

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - OFFSETORIGIN*(posIndex-centralDrone);
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle+i*deltaAngle);

    i++;
  }

  for(posIndex = 0; posIndex < centralDrone; posIndex+=2)  {
    x0 = radius*cos(i*deltaAngle);
    y0 = radius*sin(i*deltaAngle);

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - OFFSETORIGIN*(posIndex-centralDrone);
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle+i*deltaAngle);

    i++;
  }

  this->posDrones[centralDrone].x = cx;
  this->posDrones[centralDrone].y = cy;
  this->posDrones[centralDrone].z = height;
  this->posDrones[centralDrone].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);  //degrees!!
}
void Formation::cross (double droneDistance, double apertureAngle, double yawAngle, double height)
{
  this->droneDistance = droneDistance;
  this->apertureAngle = apertureAngle;
  this->yawAngle = yawAngle;
  this->height = height;
  this->currentFormation = 3;
  this->zmoved = height - TAKEOFFHEIGHT;

  int centralDrone = (this->numDrones-1)/2, i, posIndex, evenIndex;
  double cx=drones[centralDrone]->parameters.position.local.x;
  double cy=drones[centralDrone]->parameters.position.local.y;
  double cz=drones[centralDrone]->parameters.position.local.z;
  double ctheta=drones[centralDrone]->parameters.orientation.local.yaw;
  double x0=0.0, y0=0.0;

  double lineAngle = (180.0-apertureAngle)/2.0;
  lineAngle = multi_uav::utils::Math::degreesToRadians(lineAngle);

  //yawAngle is not a pointer, changes are not seen outside this function
  yawAngle = multi_uav::utils::Math::degreesToRadians(yawAngle);

  for (i=1, evenIndex=1, posIndex = centralDrone+1; i<= numDrones/2; i++, posIndex++){ //to +y
    x0 = - sin(lineAngle)*droneDistance*(evenIndex)*pow(-1,i);
    y0 = cos(lineAngle)*droneDistance*evenIndex;

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - (OFFSETORIGIN*evenIndex + OFFSETORIGIN*(i-evenIndex));
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);
    if (i%2 == 0){ //even index
      evenIndex++;
    }
  }

  for (i=1, evenIndex=1, posIndex = centralDrone-1; i<= (this->numDrones-1)/2; i++, posIndex--){ //to -y
    x0 = - sin(lineAngle)*droneDistance*(evenIndex)*pow(-1,i);
    y0 = - cos(lineAngle)*droneDistance*evenIndex;

    this->posDrones[posIndex].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
    this->posDrones[posIndex].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) + OFFSETORIGIN*evenIndex + OFFSETORIGIN*(i-evenIndex);
    this->posDrones[posIndex].z = height;
    this->posDrones[posIndex].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);
    if (i%2 == 0){ //even index
      evenIndex++;
    }
  }

  this->posDrones[centralDrone].x = cx;
  this->posDrones[centralDrone].y = cy;
  this->posDrones[centralDrone].z = height;
  this->posDrones[centralDrone].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);  //degrees!!
}
void Formation::arrow (double droneDistance, double apertureAngle, double yawAngle, double height)
{
  this->droneDistance = droneDistance;
  this->apertureAngle = apertureAngle;
  this->yawAngle = yawAngle;
  this->height = height;
  this->currentFormation = 2;
  this->zmoved = height - TAKEOFFHEIGHT;

  int centralDrone = (this->numDrones-1)/2;
  double cx=drones[centralDrone]->parameters.position.local.x;
  double cy=drones[centralDrone]->parameters.position.local.y;
  double cz=drones[centralDrone]->parameters.position.local.z;
  double ctheta=drones[centralDrone]->parameters.orientation.local.yaw;  //degrees!!
  double lineAngle = 0.0, deltaId = 0.0;

  double x0=0.0, y0=0.0;

  lineAngle = (180.0-apertureAngle)/2.0;
  lineAngle = multi_uav::utils::Math::degreesToRadians(lineAngle);
  //yawAngle is not a pointer, changes are not seen outside this function
  yawAngle = multi_uav::utils::Math::degreesToRadians(yawAngle);

  for(int i=0; i<this->numDrones; i++){
    deltaId = abs(i-centralDrone);
    if(i < centralDrone)   {
      x0 = - sin(lineAngle)*droneDistance*deltaId;
      y0 = - cos(lineAngle)*droneDistance*deltaId;
      this->posDrones[i].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
      this->posDrones[i].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) + OFFSETORIGIN*deltaId;
      this->posDrones[i].z = height;
      this->posDrones[i].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);  //degrees!!
    }
    else if(i > centralDrone) {
      x0 = - sin(lineAngle)*droneDistance*deltaId;
      y0 = + cos(lineAngle)*droneDistance*deltaId;
      this->posDrones[i].x = cx + x0*cos(yawAngle) - y0*sin(yawAngle);
      this->posDrones[i].y = cy + x0*sin(yawAngle) + y0*cos(yawAngle) - OFFSETORIGIN*deltaId;
      this->posDrones[i].z = height;
      this->posDrones[i].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);
    }
    else if(i == centralDrone)    {
      this->posDrones[i].x = cx;
      this->posDrones[i].y = cy;
      this->posDrones[i].z = height;
      this->posDrones[i].theta = multi_uav::utils::Math::radiansToDegrees(yawAngle);  //degrees!!
    }
  }
}
void Formation::line  (double droneDistance,                       double yawAngle, double height)
{ 
  this->arrow(droneDistance, 180, yawAngle, height);
  this->currentFormation = 1;
}

void Formation::updateFormation()
{
  switch (this->currentFormation)
  {
    case 1:
      this->line(this->droneDistance, this->yawAngle, this->height);
      break;

    case 2:
      this->arrow(this->droneDistance, this->apertureAngle, this->yawAngle, this->height);
      break;

    case 3:
      this->cross(this->droneDistance, this->apertureAngle, this->yawAngle, this->height);
      break;

    case 4:
      this->circle(this->droneDistance, this->yawAngle, this->height);
      break;

    default:
      break;
  }
}
////////////////////////////////// Joystick Functions  /////////////////////////////////////////
void Formation::moveJoystick            (int command)
{
  //yawAngle is not a pointer, changes are not seen outside this function
  double yawAngleRad = multi_uav::utils::Math::degreesToRadians(this->yawAngle);

  if(command == 119){  //  w - forward
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x += MOVEJM*cos(yawAngleRad);
        this->posDrones[i].y += MOVEJM*sin(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved += MOVEJM*cos(yawAngleRad);
    this->ymoved += MOVEJM*sin(yawAngleRad);
  }
  else if(command == 115){  //  s - backward
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x -= MOVEJM*cos(yawAngleRad);
        this->posDrones[i].y -= MOVEJM*sin(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved -= MOVEJM*cos(yawAngleRad);
    this->ymoved -= MOVEJM*sin(yawAngleRad);
  }

  else if(command == 100){  //  d - right
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x += MOVEJM*sin(yawAngleRad);
        this->posDrones[i].y -= MOVEJM*cos(yawAngle);
        this->posDrones[i].z = this->height;
    }
    this->xmoved += MOVEJM*sin(yawAngleRad);
    this->ymoved -= MOVEJM*cos(yawAngleRad);
  }
  else if(command == 97){  //  a - left
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x -= MOVEJM*sin(yawAngleRad);
        this->posDrones[i].y += MOVEJM*cos(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved -= MOVEJM*sin(yawAngleRad);
    this->ymoved += MOVEJM*cos(yawAngleRad);
  }

  else if(command == 102){  //  f - up
    height += UPDOWNJM;
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].z = this->height;
    }
    this->zmoved += UPDOWNJM;
  }
  else if(command == 103){  //  g - down
    height -= UPDOWNJM;
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].z = this->height;
    }
    this->zmoved -= UPDOWNJM;
  }


}
void Formation::distanceApertureJoystick(int command)
{
  //  Distance
  if(command == 111){
    this->droneDistance += DISTANCEJM;
  }
  else if(command == 112){
    this->droneDistance -= DISTANCEJM;
    if(this->droneDistance < 1.0)    {
      this->droneDistance = 1.0;
    }
  }

  //  Aperture
  if(command == 107){
    this->apertureAngle += APERTUREJM;
  }
  else if(command == 108){
    this->apertureAngle -= APERTUREJM;
  }
 this->updateFormation();
}
void Formation::rotateJoystick          (int command)
{
  //  q  -  counter clock wise
  if (command == 113){
    this->yawAngle += ROTATEJM;
    if (this->yawAngle > 180){
      this->yawAngle -= 360;
    }
  }
  //  e  -  clock wise
  else if (command == 101){
    this->yawAngle -= ROTATEJM;
    if (this->yawAngle < -180){
      this->yawAngle += 360;
    }
  }
  this->updateFormation();
}
void Formation::joystickMode            ()
{
  int command, quitJM;

  this->printJoystickControls();
  quitJM=0;

  while (!quitJM)
  {
    command = getch();
    switch (command)
    {
      //  Move
      case 119: case 97: case 115: case 100:  // w a s d
      case 102: case 103:                     // f g
        this->moveJoystick(command);
        break;

      //  Rotate
      case 101: case 113:
       this->rotateJoystick(command);
        break;

      //  Distance and Aperture
      case 111: case 112:  //o p  -  distance
      case 107: case 108:  //k l  -  aperture
        this->distanceApertureJoystick(command);
        break;

      //  Esc
      case 27:  //esc
        std::cout << " Joystick Mode closed" << std::endl;
        quitJM=1;
        break;

      //  Formations
      case 49 ... 57:
        //formationJoystick(numDrones, droneDistance, apertureAngle, yawAngle, height, oldValidFormation, 1);
        this->currentFormation = command-48;
        this->updateFormation();  // 49-48 = 1
        break;


      default:
        break;
    }

    //printJoystickcommands(command);
    std::system("clear");
    this->printJoystickControls();
    this->printCurrentFormationInfo();
  }

  ros::shutdown();
}

////////////////////////////////////  Auto Functions  ////////////////////////////////////
void Formation::moveAuto                 (double distance, int command)
{
  //yawAngle is not a pointer, changes are not seen outside this function
  double yawAngleRad = multi_uav::utils::Math::degreesToRadians(this->yawAngle);


  if(command == 119){  //  w - forward
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x += distance*cos(yawAngleRad);
        this->posDrones[i].y += distance*sin(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved += distance*cos(yawAngleRad);
    this->ymoved += distance*sin(yawAngleRad);
  }
  else if(command == 115){  //  s - backward
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x -= distance*cos(yawAngleRad);
        this->posDrones[i].y -= distance*sin(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved -= distance*cos(yawAngleRad);
    this->ymoved -= distance*sin(yawAngleRad);
  }

  else if(command == 100){  //  d - right
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x += distance*sin(yawAngleRad);
        this->posDrones[i].y -= distance*cos(yawAngle);
        this->posDrones[i].z = this->height;
    }
    this->xmoved += distance*sin(yawAngleRad);
    this->ymoved -= distance*cos(yawAngleRad);
  }
  else if(command == 97){  //  a - left
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].x -= distance*sin(yawAngleRad);
        this->posDrones[i].y += distance*cos(yawAngleRad);
        this->posDrones[i].z = this->height;
    }
    this->xmoved -= distance*sin(yawAngleRad);
    this->ymoved += distance*cos(yawAngleRad);
  }

  else if(command == 102){  //  f - up
    height += UPDOWNJM;
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].z = this->height;
    }
    this->zmoved += UPDOWNJM;
  }
  else if(command == 103){  //  g - down
    height -= UPDOWNJM;
    for(int i=0; i<numDrones; i++){
        this->posDrones[i].z = this->height;
    }
    this->zmoved -= UPDOWNJM;
  }


}
void Formation::moveForwardFormation     (double distance)
{
  this->moveAuto(distance, 119);
}
void Formation::moveBackwardFormation    (double distance)
{
  this->moveAuto(distance, 115);
}
void Formation::moveLeftFormation        (double distance)
{
  this->moveAuto(distance, 97);
}
void Formation::moveRightFormation       (double distance)
{
  this->moveAuto(distance, 100);
}
void Formation::moveUpFormation          (double distance)
{
  this->zmoved += distance;
  this->moveAuto(distance, 102);
}
void Formation::moveDownFormation        (double distance)
{
  this->zmoved -= distance;
  this->moveAuto(distance, 103);
}
void Formation::setDroneDistanceFormation(double droneDistance)
{
    this->droneDistance = droneDistance;
    this->updateFormation();
}
void Formation::setApertureAngleFormation(double apertureAngle)
{
    this->apertureAngle = apertureAngle;
    this->updateFormation();
}
void Formation::setYawAngleFormation     (double yawAngle)
{
    this->yawAngle = yawAngle;
    this->updateFormation();
}


}
