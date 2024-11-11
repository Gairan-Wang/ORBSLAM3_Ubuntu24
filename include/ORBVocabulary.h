#ifndef ORB_VOCABULARY_H
#define ORB_VOCABULARY_H

#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>

namespace ORB_SLAM3 {
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;
} // namespace ORB_SLAM3

#endif // ORB_VOCABULARY_H