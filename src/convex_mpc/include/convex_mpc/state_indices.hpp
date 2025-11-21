#pragma once

/**
 * @file state_indices.hpp
 * @brief State index enums to lock state ordering everywhere
 * 
 * State order: [roll, pitch, yaw, x, y, z, roll_dot, pitch_dot, yaw_dot, x_dot, y_dot, z_dot, gravity]
 * This matches: [R, P, Y, X, YL, Z, Rd, Pd, Yd, Xd, YLd, Zd, g]
 */

enum StateIdx {
    IDX_R = 0,   // roll
    IDX_P = 1,   // pitch
    IDX_Y = 2,   // yaw
    IDX_X = 3,   // x
    IDX_YL = 4,  // y (avoid name clash with yaw)
    IDX_Z = 5,   // z
    STATE_DIM = 6  // Pose dimension
};

enum StateDotIdx {
    IDX_Rd = 6,   // roll_dot
    IDX_Pd = 7,   // pitch_dot
    IDX_Yd = 8,   // yaw_dot
    IDX_Xd = 9,   // x_dot
    IDX_YLd = 10, // y_dot (avoid name clash)
    IDX_Zd = 11,  // z_dot
    FULL_STATE_DIM = 12  // Without gravity
};

enum StateFullIdx {
    IDX_G = 12,      // gravity
    FULL_STATE_WITH_G_DIM = 13  // Full state including gravity
};

