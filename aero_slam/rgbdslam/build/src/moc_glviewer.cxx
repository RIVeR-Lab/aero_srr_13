/****************************************************************************
** Meta object code from reading C++ file 'glviewer.h'
**
** Created: Sat May 25 23:16:35 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/glviewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'glviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GLViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      24,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      10,    9,    9,    9, 0x05,

 // slots: signature, parameters, type, tag, flags
      54,   48,    9,    9, 0x0a,
      72,   48,    9,    9, 0x0a,
      90,   48,    9,    9, 0x0a,
     114,  108,    9,    9, 0x0a,
     156,  137,    9,    9, 0x0a,
     183,  180,    9,    9, 0x0a,
     206,  180,    9,    9, 0x0a,
     228,  180,    9,    9, 0x0a,
     248,  180,    9,    9, 0x0a,
     269,  180,    9,    9, 0x0a,
     289,  180,    9,    9, 0x0a,
     311,  180,    9,    9, 0x0a,
     334,  180,    9,    9, 0x0a,
     359,  180,    9,    9, 0x0a,
     387,  180,    9,    9, 0x0a,
     419,  406,    9,    9, 0x0a,
     483,  462,    9,    9, 0x0a,
     586,  575,    9,    9, 0x0a,
     633,  623,    9,    9, 0x0a,
     673,    9,    9,    9, 0x0a,
     690,    9,    9,    9, 0x0a,
     698,    9,    9,    9, 0x0a,
     728,  720,    9,    9, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GLViewer[] = {
    "GLViewer\0\0cloudRendered(const pointcloud_type*)\0"
    "angle\0setXRotation(int)\0setYRotation(int)\0"
    "setZRotation(int)\0shift\0setStereoShift(double)\0"
    "rot_step_in_degree\0setRotationGrid(double)\0"
    "on\0toggleFollowMode(bool)\0"
    "toggleShowEdges(bool)\0toggleShowIDs(bool)\0"
    "toggleShowGrid(bool)\0toggleShowTFs(bool)\0"
    "toggleShowPoses(bool)\0toggleShowClouds(bool)\0"
    "toggleShowFeatures(bool)\0"
    "toggleBackgroundColor(bool)\0"
    "toggleStereo(bool)\0pc,transform\0"
    "addPointCloud(pointcloud_type*,QMatrix4x4)\0"
    "feature_locations_3d\0"
    "addFeatures(const std::vector<Eigen::Vector4f,Eigen::aligned_allocator"
    "<Eigen::Vector4f> >*)\0"
    "transforms\0updateTransforms(QList<QMatrix4x4>*)\0"
    "edge_list\0setEdges(const QList<QPair<int,int> >*)\0"
    "deleteLastNode()\0reset()\0toggleTriangulation()\0"
    "filname\0drawToPS(QString)\0"
};

void GLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GLViewer *_t = static_cast<GLViewer *>(_o);
        switch (_id) {
        case 0: _t->cloudRendered((*reinterpret_cast< const pointcloud_type*(*)>(_a[1]))); break;
        case 1: _t->setXRotation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->setYRotation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->setZRotation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->setStereoShift((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->setRotationGrid((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->toggleFollowMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->toggleShowEdges((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->toggleShowIDs((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->toggleShowGrid((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->toggleShowTFs((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->toggleShowPoses((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->toggleShowClouds((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 13: _t->toggleShowFeatures((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->toggleBackgroundColor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->toggleStereo((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->addPointCloud((*reinterpret_cast< pointcloud_type*(*)>(_a[1])),(*reinterpret_cast< QMatrix4x4(*)>(_a[2]))); break;
        case 17: _t->addFeatures((*reinterpret_cast< const std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >*(*)>(_a[1]))); break;
        case 18: _t->updateTransforms((*reinterpret_cast< QList<QMatrix4x4>*(*)>(_a[1]))); break;
        case 19: _t->setEdges((*reinterpret_cast< const QList<QPair<int,int> >*(*)>(_a[1]))); break;
        case 20: _t->deleteLastNode(); break;
        case 21: _t->reset(); break;
        case 22: _t->toggleTriangulation(); break;
        case 23: _t->drawToPS((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GLViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GLViewer::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_GLViewer,
      qt_meta_data_GLViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GLViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GLViewer))
        return static_cast<void*>(const_cast< GLViewer*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int GLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 24)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 24;
    }
    return _id;
}

// SIGNAL 0
void GLViewer::cloudRendered(pointcloud_type const * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
