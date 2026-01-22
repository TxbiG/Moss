#include <Moss/Moss_Renderer.h>





// Camera2D

Camera2::Camera2(Float2 position) : position(position), zoom(1.0f), rotation(0.0f), viewProj(Mat44::sIdentity()) { }

void Camera2::update(float width, float height) { 
    Mat44 translation = Mat44::sTranslation(Vec3(-position.x, -position.y, 0.0f));
    Mat44 rotationMat = Mat44::sRotationZ(-rotation);
    Mat44 scaleMat = Mat44::sScale(Vec3(zoom, zoom, 1));
    Mat44 view = scaleMat * rotationMat * translation;

    Mat44 proj = ortho(0.0f, width, height, 0.0f, -1.0f, 1.0f);
    viewProj = proj * view; 
}

Mat44 Camera2::getViewProjectionMatrix() const { return viewProj; }



// Camera3D


Camera3::Camera3(Vec3 position) : position(position) { }

Mat44 Camera3::getViewProjectionMatrix() const { return viewProj; }

void Camera3::update(float width, float height) { 
    Mat44 view = Mat44::sLookAt(position, position + target, up); 
    Mat44 proj = Mat44::sPerspective(fov, (height != 0) ? width / height : 1.0f, nearPlane, farPlane);
    viewProj = proj * view; 
}