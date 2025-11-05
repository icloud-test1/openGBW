#pragma once

class MenuItem {
    public:
        int id;
        bool selected;
        char menuName[16];
        double increment;
        double *value;
};