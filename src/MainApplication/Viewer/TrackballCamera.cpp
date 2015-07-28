#include <MainApplication/Viewer/TrackballCamera.hpp>

#include <cmath>
#include <QmessageBox>

#include <Core/Event/MouseEvent.hpp>
#include <Core/Event/KeyEvent.hpp>
#include <Engine/Renderer/Camera/Camera.hpp>

namespace Ra
{

Gui::TrackballCamera::TrackballCamera(uint width, uint height)
    : CameraInterface(width, height)
    , m_cameraPosition(0, 0, 1)
    , m_trackballCenter(0, 0, 0)
    , m_quickCameraModifier(1.0)
    , m_cameraRotateMode(false)
    , m_cameraPanMode(false)
    , m_cameraZoomMode(false)
{
    resetCamera();
}

Gui::TrackballCamera::~TrackballCamera()
{
}

void Gui::TrackballCamera::resetCamera()
{
    m_camera->setFrame(Core::Transform::Identity());
    m_trackballCenter = Core::Vector3::Zero();
    m_camera->setPosition(Core::Vector3(0, 0, 1));
    m_distanceToCenter = (m_cameraPosition - m_trackballCenter).norm();
    updatePhiTheta();

    m_viewIsDirty = true;

    emit cameraPositionChanged(m_camera->getPosition());
    emit cameraTargetChanged(m_trackballCenter);
}

bool Gui::TrackballCamera::handleMousePressEvent(QMouseEvent* event)
{
    // Whole manipulation is done with left button and modifiers
    if (event->button() != Qt::LeftButton)
    {
        return false;
    }

    bool handled = false;
    m_lastMouseX = event->pos().x();
    m_lastMouseY = event->pos().y();

    if (event->modifiers().testFlag(Qt::NoModifier))
    {
        m_cameraRotateMode = true;
        handled = true;
    }

    if (event->modifiers().testFlag(Qt::ControlModifier))
    {
        m_cameraPanMode = true;
        handled = true;
    }

    if (event->modifiers().testFlag(Qt::ShiftModifier))
    {
        m_cameraZoomMode = true;
        handled = true;
    }

    return handled;
}

bool Gui::TrackballCamera::handleMouseMoveEvent(QMouseEvent *event)
{
    Scalar dx = (event->pos().x() - m_lastMouseX) / m_width;
    Scalar dy = (event->pos().y() - m_lastMouseY) / m_height;

    if (event->modifiers().testFlag(Qt::AltModifier))
    {
        m_quickCameraModifier = 10.0;
    }
    else
    {
        m_quickCameraModifier = 1.0;
    }

    if (m_cameraRotateMode)
    {
        handleCameraRotate(dx, dy);
    }

    if (m_cameraPanMode)
    {
        handleCameraPan(dx, dy);
    }

    if (m_cameraZoomMode)
    {
        handleCameraZoom(dx, dy);
    }

    m_lastMouseX = event->pos().x();
    m_lastMouseY = event->pos().y();

    emit cameraPositionChanged(m_camera->getPosition());
    emit cameraTargetChanged(m_trackballCenter);

    return true;
}

bool Gui::TrackballCamera::handleMouseReleaseEvent(QMouseEvent *event)
{
    m_cameraRotateMode = false;
    m_cameraPanMode = false;
    m_cameraZoomMode = false;
    m_quickCameraModifier = 1.0;

    return true;
}

bool Gui::TrackballCamera::handleKeyPressEvent(QKeyEvent*)
{
    // No keyboard manipulation
    return false;
}

bool Gui::TrackballCamera::handleKeyReleaseEvent(QKeyEvent*)
{
    return false;
}

void Gui::TrackballCamera::setCameraPosition(const Core::Vector3& position)
{

    if (position == m_trackballCenter)
    {
        QMessageBox::warning(nullptr,
            "Error", "Position cannot be set to target point");
        return;
    }

    m_camera->setPosition(position);
    m_camera->setDirection(m_trackballCenter - position);

    m_viewIsDirty = true;
    updatePhiTheta();

    emit cameraPositionChanged(m_camera->getPosition());
    emit cameraTargetChanged(m_trackballCenter);
}

void Gui::TrackballCamera::setCameraTarget(const Core::Vector3 &target)
{
    // FIXME(Charly): Bug when we set the target point, then rotate the camera again
    //return;

    m_trackballCenter = target;
    m_camera->setDirection(target - m_camera->getPosition());
    m_viewIsDirty = true;
    updatePhiTheta();

    emit cameraPositionChanged(m_camera->getPosition());
    emit cameraTargetChanged(m_trackballCenter);
}

void Gui::TrackballCamera::moveCameraToFitAabb(const Core::Aabb& aabb)
{
    CameraInterface::moveCameraToFitAabb(aabb);
    m_trackballCenter = aabb.center();
    m_distanceToCenter = std::min((m_camera->getPosition() -m_trackballCenter).norm(), Scalar(100.0));
    updatePhiTheta();

    emit cameraPositionChanged(m_camera->getPosition());
    emit cameraTargetChanged(m_trackballCenter);
}

void Gui::TrackballCamera::handleCameraRotate(Scalar dx, Scalar dy)
{
    Scalar x = dx * m_cameraSensitivity * m_quickCameraModifier;
    Scalar y = -dy * m_cameraSensitivity * m_quickCameraModifier;
    constexpr Scalar Pi = M_PI;

    Scalar phi = m_phi + x; // Keep phi between -pi and pi
    Scalar theta = std::min(std::max(m_theta + y, Scalar(0.0)), Pi);

    Scalar dphi = phi - m_phi;
    Scalar dtheta = theta - m_theta;

    const Core::Vector3 C = m_trackballCenter;
    const Core::Vector3 P0 = m_camera->getPosition();
    const Scalar r = (C-P0).norm();
    // Compute new camera position, on the sphere of radius r centered on C
    Scalar px = C.x() + r * std::cos(phi) * std::sin(theta);
    Scalar py = C.y() + r * std::cos(theta);
    Scalar pz = C.z() + r * std::sin(phi) * std::sin(theta);

    // Compute the translation from old pos to new pos
    Core::Vector3 P(px, py, pz);
    Core::Vector3 t(P - P0);

    // Translate the camera given this translation
    Core::Transform T(Core::Transform::Identity());
    T.translation() = t;

    // Rotate the camera so that it points to the center
    Core::Transform R1(Core::Transform::Identity());
    Core::Transform R2(Core::Transform::Identity());

    Core::Vector3 U = Core::Vector3(0, 1, 0);
    Core::Vector3 R = -m_camera->getRightVector().normalized();

    R1 = Core::AngleAxis(-dphi, U);
    R2 = Core::AngleAxis(-dtheta, R);

    m_camera->applyTransform(T * R1 * R2);

    m_phi = phi;
    m_theta = theta;
    m_viewIsDirty = true;
}

void Gui::TrackballCamera::handleCameraPan(Scalar dx, Scalar dy)
{
    Scalar x = dx * m_distanceToCenter * m_cameraSensitivity * m_quickCameraModifier;
    Scalar y = dy * m_distanceToCenter * m_cameraSensitivity * m_quickCameraModifier;
    // Move camera and trackball center, keep the distance to the center
    Core::Vector3 R = -m_camera->getRightVector();
    Core::Vector3 U = m_camera->getUpVector();

    Core::Transform T(Core::Transform::Identity());
    Core::Vector3 t = x * R + y * U;
    T.translate(t);

    m_camera->applyTransform(T);
    m_trackballCenter += t;

    m_viewIsDirty = true;
}

void Gui::TrackballCamera::handleCameraZoom(Scalar dx, Scalar dy)
{
    Scalar y = dy * m_distanceToCenter * m_cameraSensitivity * m_quickCameraModifier;
    Core::Vector3 F = m_camera->getDirection();

    Core::Transform T(Core::Transform::Identity());
    Core::Vector3 t = y * F;
    T.translate(t);

    m_camera->applyTransform(T);

    m_distanceToCenter = std::min(Scalar(100.0), (m_camera->getPosition() - m_trackballCenter).norm());

    m_viewIsDirty = true;
}

void Gui::TrackballCamera::updatePhiTheta()
{
    const Core::Vector3 P = m_camera->getPosition();
    const Core::Vector3 C = m_trackballCenter;
    const Scalar r = m_distanceToCenter;

    m_theta = std::acos((P.y() - C.y()) / r);
    m_phi   = std::atan((P.z() - C.z()) / (P.x() - C.x()));
    CORE_ASSERT(isfinite(m_theta) && isfinite(m_phi), "Error in trackball camera");
}

} // namespace Ra
