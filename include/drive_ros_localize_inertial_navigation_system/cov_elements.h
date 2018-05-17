#ifndef COV_ELEMENTS_H
#define COV_ELEMENTS_H
/*
 * Covariance Matrix Element Definitions
 *
 */

namespace CovElem{

// linear [9]
namespace lin{
  enum el{
    linX_linX = 0,
    linX_linY = 1,
    linX_linZ = 2,
    linY_linX = 3,
    linY_linY = 4,
    linY_linZ = 5,
    linZ_linX = 6,
    linZ_linY = 7,
    linZ_linZ = 8
  };
}


// angular [9]
namespace ang{
  enum el{
    angX_angX = 0,
    angX_angY = 1,
    angX_angZ = 2,
    angY_angX = 3,
    angY_angY = 4,
    angY_angZ = 5,
    angZ_angX = 6,
    angZ_angY = 7,
    angZ_angZ = 8
  };
}

// linear + angular [36]
namespace lin_ang{
enum el{
    linX_linX = 0,
    linX_linY = 1,
    linX_linZ = 2,
    linX_angX = 3,
    linX_angY = 4,
    linX_angZ = 5,
    linY_linX = 6,
    linY_linY = 7,
    linY_linZ = 8,
    linY_angX = 9,
    linY_angY = 10,
    linY_angZ = 11,
    linZ_linX = 12,
    linZ_linY = 13,
    linZ_linZ = 14,
    linZ_angX = 15,
    linZ_angY = 16,
    linZ_angZ = 17,
    angX_linX = 18,
    angX_linY = 19,
    angX_linZ = 20,
    angX_angX = 21,
    angX_angY = 22,
    angX_angZ = 23,
    angY_linX = 24,
    angY_linY = 25,
    angY_linZ = 26,
    angY_angX = 27,
    angY_angY = 28,
    angY_angZ = 29,
    angZ_linX = 30,
    angZ_linY = 31,
    angZ_linZ = 32,
    angZ_angX = 33,
    angZ_angY = 34,
    angZ_angZ = 35,
  };
}


} // CovElements


#endif // COV_ELEMENTS_H
