#ifndef APRILTAGS_TAGFAMILY_H
#define APRILTAGS_TAGFAMILY_H

#include <string>
#include <map>
#include <vector>
#include <stdexcept>
#include <iostream>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"

typedef std::map<std::string, std::pair<apriltag_family_t *(*)(), void (*)(apriltag_family_t *)>> familyMapping;

class TagFamily {
public:
    TagFamily() = delete;
    TagFamily(const TagFamily&) = delete;
    explicit TagFamily(std::string& name);
    ~TagFamily();

    static std::vector<std::string> getFamilyNames();

    apriltag_family_t* at_family;

private:
    static familyMapping getFamilies();
    void (*m_destructor)(apriltag_family_t*);
};


#endif //APRILTAGS_TAGFAMILY_H
