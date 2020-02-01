
#include <Arduino.h>
#include "zippies/paths/Routine.h"


/*
#if ZIPPY_ID == 0
Movement MOVEMENT0[]
{
  { MovementType::Arc  ,        30.0d,  M_PI         },
  { MovementType::Arc  ,        30.0d,  M_PI         },
  { MovementType::Arc  ,      -150.0d, -M_PI         },
  { MovementType::Arc  ,      -150.0d, -M_PI         },
};
#elif ZIPPY_ID == 1
Movement MOVEMENT0[]
{
  { MovementType::Arc  ,        90.0d,  M_PI         },
  { MovementType::Arc  ,        90.0d,  M_PI         },
  { MovementType::Arc  ,       -90.0d, -M_PI         },
  { MovementType::Arc  ,       -90.0d, -M_PI         },
};
#else
Movement MOVEMENT0[]
{
  { MovementType::Arc  ,       150.0d,  M_PI         },
  { MovementType::Arc  ,       150.0d,  M_PI         },
  { MovementType::Arc  ,       -30.0d, -M_PI         },
  { MovementType::Arc  ,       -30.0d, -M_PI         },
};
#endif

PathSegment ROUTINE0[]
{
  { 20000, 0.25d, 0.25d, sizeof(MOVEMENT0) / sizeof(Movement), MOVEMENT0, 0 },
};
*/

/*
Movement MOVEMENT[]
{
  { MovementType::Arc  ,       -50.0d, -M_PI_2       },
  { MovementType::Arc  ,       -50.0d,  M_PI         },
  { MovementType::Arc  ,       -50.0d, -M_PI_2       },
  { MovementType::Arc  ,        50.0d,  M_PI_2       },
  { MovementType::Arc  ,        50.0d, -M_PI         },
  { MovementType::Arc  ,        50.0d,  M_PI_2       },
};
*/

/*
Movement MOVEMENT[]
{
  { MovementType::Arc  ,       -50.0d, -M_PI_2       },
  { MovementType::Arc  ,       -50.0d,  M_PI         },
  { MovementType::Arc  ,       -50.0d, -M_PI_2       },
  { MovementType::Arc  ,        50.0d,  M_PI_2       },
  { MovementType::Arc  ,        50.0d, -M_PI         },
  { MovementType::Arc  ,        50.0d,  M_PI_2       },
};

PathSegment DEFAULT_ROUTINE[]
{
  { 20000, 0.25d, 0.25d, sizeof(MOVEMENT) / sizeof(Movement), MOVEMENT, 0 },
};
*/

// #define M2_PI 6.283185307179586d

// Command ROUTINE[] = {
  /*
  {             3400, CommandArc,            -300.0d,      -M_PI        },
  {             3400, CommandArc,            -300.0d,      -M_PI        },
  {             3400, CommandArc,             300.0d,       M_PI        },
  {             3400, CommandArc,             300.0d,       M_PI        },
  */
  // {             2800, CommandArc,            -300.0d,      -M_PI        },
  // {             2800, CommandArc,            -300.0d,      -M_PI        },
  // {             2800, CommandArc,             300.0d,       M_PI        },
  // {             2800, CommandArc,             300.0d,       M_PI        },
  // {             2000, CommandPause                                      },
  // {             2000, CommandArc,            -200.0d,      -M_PI        },
  // {             2000, CommandArc,            -200.0d,      -M_PI        },
  // {             2000, CommandArc,             200.0d,       M_PI        },
  // {             2000, CommandArc,             200.0d,       M_PI        },
  // */

  /*
  {                0, CommandSync                                       },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  {             2000, CommandTurn,              M_PI                    },
  */

  /*
  //4 seconds to move into starting position
  {             8000, CommandMoveTo,            0.0d,   -10.0d,    0.0d },
  //sync with lighthouse
  {                0, CommandSync                                       },

  //turning left
  {             5000, CommandArc,             -40.0d,      -M_PI        }, // +40
  {             4750, CommandArc,             -60.0d,      -M_PI        }, // -20
  {             4500, CommandArc,             -80.0d,      -M_PI        }, // +60
  {             4250, CommandArc,            -100.0d,      -M_PI        }, // -40
  {             4000, CommandArc,            -120.0d,      -M_PI        }, // +80
  {             3750, CommandArc,            -140.0d,      -M_PI        }, // -60
  {             3500, CommandArc,            -160.0d,      -M_PI        }, //+100
  {             3250, CommandArc,            -180.0d,      -M_PI        }, // -80
  {             3000, CommandArc,            -200.0d,      -M_PI        }, //+120
  {             2750, CommandArc,            -220.0d,      -M_PI        }, //-100

  {             2500, CommandArc,            -240.0d,      -M_PI        }, //+140
  {             2250, CommandArc,            -240.0d,      -M_PI        }, //+140
  {             2000, CommandArc,            -240.0d,      -M_PI        }, //+140
  {             1750, CommandArc,            -240.0d,      -M_PI        }, //+140

  // {             1600, CommandArc,            -200.0d,      -M_PI        }, //+140
  // {             1550, CommandArc,            -180.0d,      -M_PI        }, //+140
  // {             1600, CommandArc,            -200.0d,      -M_PI        }, //+140
  // {             1750, CommandArc,            -240.0d,      -M_PI        }, //+140

  {             2000, CommandArc,            -240.0d,      -M_PI        }, //+140
  {             2250, CommandArc,            -240.0d,      -M_PI        }, //+140
  {             2500, CommandArc,            -240.0d,      -M_PI        }, //+140

  {             2250, CommandArc,            -220.0d,      -M_PI        }, // -80
  {             2000, CommandArc,            -200.0d,      -M_PI        }, //+120
  {             1500, CommandArc,            -140.0d,      -M_PI        }, // -40
  {             1000, CommandArc,            -100.0d,      -M_PI        }, // +60
  {              750, CommandArc,             -60.0d,      -M_PI        }, //   0

  //turning right
  {             5000, CommandArc,              40.0d,       M_PI        }, // +40
  {             4750, CommandArc,              60.0d,       M_PI        }, // -20
  {             4500, CommandArc,              80.0d,       M_PI        }, // +60
  {             4250, CommandArc,             100.0d,       M_PI        }, // -40
  {             4000, CommandArc,             120.0d,       M_PI        }, // +80
  {             3750, CommandArc,             140.0d,       M_PI        }, // -60
  {             3500, CommandArc,             160.0d,       M_PI        }, //+100
  {             3250, CommandArc,             180.0d,       M_PI        }, // -80
  {             3000, CommandArc,             200.0d,       M_PI        }, //+120
  {             2750, CommandArc,             220.0d,       M_PI        }, //-100

  {             2500, CommandArc,             240.0d,       M_PI        }, //+140
  {             2250, CommandArc,             240.0d,       M_PI        }, //+140
  {             2000, CommandArc,             240.0d,       M_PI        }, //+140
  {             1750, CommandArc,             240.0d,       M_PI        }, //+140

  // {             1600, CommandArc,             200.0d,       M_PI        }, //+140
  // {             1550, CommandArc,             180.0d,       M_PI        }, //+140
  // {             1600, CommandArc,             200.0d,       M_PI        }, //+140
  // {             1750, CommandArc,             240.0d,       M_PI        }, //+140

  {             2000, CommandArc,             240.0d,       M_PI        }, //+140
  {             2250, CommandArc,             240.0d,       M_PI        }, //+140
  {             2500, CommandArc,             240.0d,       M_PI        }, //+140

  {             2250, CommandArc,             220.0d,       M_PI        }, // -80
  {             2000, CommandArc,             200.0d,       M_PI        }, //+120
  {             1500, CommandArc,             140.0d,       M_PI        }, // -40
  {             1000, CommandArc,             100.0d,       M_PI        }, // +60
  {              750, CommandArc,              60.0d,       M_PI        }, //   0
  */

  /*
  {             8000, CommandMoveTo,            0.0d,     0.0d,    0.0d },
  {                0, CommandSync                                       },

  //dance forward to right
  { TIMING_BEATS_0_5, CommandArc,             100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },

  //dance backward to left
  { TIMING_BEATS_0_5, CommandArc,            -100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,            -100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,            -100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,            -100.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  // */

  /*
  //4 seconds to move into starting position
  {             2000, CommandMoveTo,          -50.0d,   500.0d,    M_PI },
  //sync with lighthouse
  {                0, CommandSync                                       },

  //intro
  { TIMING_BEATS_3_0, CommandPause                                      },
  { TIMING_BEATS_4_0, CommandMoveTo,           50.0d,     0.0d,    M_PI },
  { TIMING_BEATS_1_0, CommandTurnTo,         -M_PI_2                    },
  { TIMING_BEATS_7_0, CommandPause                                      },
  { TIMING_BEATS_0_5, CommandTurnTo,            0.0d                    },
  { TIMING_BEATS_0_5, CommandPause                                      },

  { TIMING_BEATS_0_5, CommandArc,              75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },

  //dance backward to left
  { TIMING_BEATS_0_5, CommandArc,             -75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -75.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },

  //dance forward to left
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,             -25.0d,   M_PI_4          },

  //dance backward to right
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              25.0d,   M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              50.0d,  -M_PI_4          },
  { TIMING_BEATS_0_5, CommandArc,              25.0d,   M_PI_4          },

  //clockwise circle in reverse
  { TIMING_BEATS_1_5, CommandArc,             -75.0d, 2.0d*M_PI_34      },
  { TIMING_BEATS_1_0, CommandMoveTo,          500.0d,    50.0d,    0.0d },

  //currently offscreen; rush around to stage right entrance point
  { TIMING_BEATS_5_0, CommandArc,            -500.0d,    -M_PI          },
  { TIMING_BEATS_1_0, CommandTurn,           -M_PI_2                    },

  //enter from stage right to center
  { TIMING_BEATS_2_5, CommandMoveTo,            0.0d,     0.0d,  M_PI_2 },
  { TIMING_BEATS_1_0, CommandTurn,        -(2.0d*M_PI) - M_PI_2         },
  { TIMING_BEATS_0_5, CommandPause                                      },

  {             2000, CommandPause                                      },
  // */
// };

// int ROUTINE_POSITION_COUNT = (int)(sizeof(ROUTINE) / sizeof(Command));
