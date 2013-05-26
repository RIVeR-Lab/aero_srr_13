/****************************************************************************
** Meta object code from reading C++ file 'ros_service_ui.h'
**
** Created: Tue May 14 18:43:43 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/ros_service_ui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ros_service_ui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RosUi[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      15,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      15,    6,    6,    6, 0x05,
      29,    6,    6,    6, 0x05,
      49,    6,    6,    6, 0x05,
      70,    6,    6,    6, 0x05,
      84,    6,    6,    6, 0x05,
     102,    6,    6,    6, 0x05,
     127,  118,    6,    6, 0x05,
     150,  118,    6,    6, 0x05,
     174,  118,    6,    6, 0x05,
     199,  118,    6,    6, 0x05,
     235,  221,    6,    6, 0x05,
     275,  265,    6,    6, 0x05,
     294,    6,    6,    6, 0x05,
     318,    6,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
     334,    6,    6,    6, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RosUi[] = {
    "RosUi\0\0reset()\0togglePause()\0"
    "toggleMapping(bool)\0toggleBagRecording()\0"
    "getOneFrame()\0deleteLastFrame()\0"
    "sendAllClouds()\0filename\0"
    "saveAllClouds(QString)\0saveOctomapSig(QString)\0"
    "saveAllFeatures(QString)\0saveG2OGraph(QString)\0"
    "file_basename\0saveIndividualClouds(QString)\0"
    "max_depth\0setMaxDepth(float)\0"
    "saveTrajectory(QString)\0optimizeGraph()\0"
    "sendFinished()\0"
};

void RosUi::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RosUi *_t = static_cast<RosUi *>(_o);
        switch (_id) {
        case 0: _t->reset(); break;
        case 1: _t->togglePause(); break;
        case 2: _t->toggleMapping((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->toggleBagRecording(); break;
        case 4: _t->getOneFrame(); break;
        case 5: _t->deleteLastFrame(); break;
        case 6: _t->sendAllClouds(); break;
        case 7: _t->saveAllClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->saveOctomapSig((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->saveAllFeatures((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->saveG2OGraph((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->saveIndividualClouds((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: _t->setMaxDepth((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 13: _t->saveTrajectory((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 14: _t->optimizeGraph(); break;
        case 15: _t->sendFinished(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData RosUi::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject RosUi::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RosUi,
      qt_meta_data_RosUi, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RosUi::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RosUi::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RosUi::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RosUi))
        return static_cast<void*>(const_cast< RosUi*>(this));
    return QObject::qt_metacast(_clname);
}

int RosUi::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void RosUi::reset()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void RosUi::togglePause()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void RosUi::toggleMapping(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void RosUi::toggleBagRecording()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void RosUi::getOneFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void RosUi::deleteLastFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}

// SIGNAL 6
void RosUi::sendAllClouds()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}

// SIGNAL 7
void RosUi::saveAllClouds(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void RosUi::saveOctomapSig(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void RosUi::saveAllFeatures(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void RosUi::saveG2OGraph(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void RosUi::saveIndividualClouds(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void RosUi::setMaxDepth(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void RosUi::saveTrajectory(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void RosUi::optimizeGraph()
{
    QMetaObject::activate(this, &staticMetaObject, 14, 0);
}
QT_END_MOC_NAMESPACE
