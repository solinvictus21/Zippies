
#include "ZippyPathSet.h"

#define M2_PI_34 4.712388980384690d

PathDefinition synchronizedPath[] = {
  //full circles synchronized
  { PathDefinitionType::Arc  ,                200.0d,  M2_PI      },
  { PathDefinitionType::Arc  ,               -180.0d, -M2_PI      },
  { PathDefinitionType::Arc  ,                150.0d,  M2_PI      },
  { PathDefinitionType::Arc  ,               -120.0d, -M2_PI      },
  { PathDefinitionType::Arc  ,                100.0d,  M2_PI      },

  //half-arcs synchronized
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M2_PI      },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M2_PI      },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },

  //turn to the start of the follow path
  { PathDefinitionType::Arc  ,                -90.0d, -M2_PI_34   },
  { PathDefinitionType::Move ,                 90.0d              },
  { PathDefinitionType::Arc  ,                -45.0d, -M_PI       },
};
const int synchronizedPathLength = sizeof(synchronizedPath) / sizeof(PathDefinition);

PathDefinition followPath[] = {
  //figure 8 on the left
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M2_PI      },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },

  //exit out of figure 8 toward the south
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M2_PI_34   },

  //S-curve toward the right
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI_2     },

  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI_34   },
  // -90
  { PathDefinitionType::Move ,   FOLLOW_RADIUS_SMALL              },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-270
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  //-450
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-270
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  //-450

  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-630
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },
  //-630
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-450
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  //-270
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-450
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  //-270
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  //-450

  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },

  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI_2     },

  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,   M_PI      },
};
const int followPathLength = sizeof(followPath) / sizeof(PathDefinition);
