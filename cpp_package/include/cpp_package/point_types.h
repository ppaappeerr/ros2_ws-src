#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#define PCL_NO_PRECOMPILE // PCL 컴파일 시, 직접 정의한 타입을 사용하도록 설정

#include <pcl/point_types.h>

// FAST-LIO가 요구하는 필드를 가진 우리만의 포인트 구조체 정의
struct PointLIO
{
    PCL_ADD_POINT4D; // x, y, z, padding 필드 추가
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 메모리 정렬 보장
} EIGEN_ALIGN16; // 16바이트 정렬

// PCL에 우리 커스텀 포인트를 등록하는 과정
POINT_CLOUD_REGISTER_POINT_STRUCT (PointLIO,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint8_t, tag, tag)
                                   (uint8_t, line, line)
                                   (double, timestamp, timestamp)
)
#endif // POINT_TYPES_H