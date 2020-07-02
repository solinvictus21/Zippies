
#include "zippies/config/ZippyDefaultPaths.h"

PathDefinition synchronizedPath[] = {
  //full circles synchronized
  { PathDefinitionType::Arc  ,                200.0,  M2_PI      },
  { PathDefinitionType::Arc  ,               -180.0, -M2_PI      },
  { PathDefinitionType::Arc  ,                150.0,  M2_PI      },
  { PathDefinitionType::Arc  ,               -120.0, -M2_PI      },
  { PathDefinitionType::Arc  ,                100.0,  M2_PI      },

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
  { PathDefinitionType::Arc  ,                -90.0, -M2_PI_34   },
  { PathDefinitionType::Move ,                 90.0              },
  { PathDefinitionType::Arc  ,                -45.0, -M_PI       },
};
const int synchronizedPathLength = sizeof(synchronizedPath) / sizeof(PathDefinition);

PathDefinition followPath[] = {
  //figure 8 on the left
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M2_PI      },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },

  //exit figure 8 toward south
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M2_PI_34   },

  //medium half circles east
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,  M_PI       },

  //turn around
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI_2     },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI_34   },

  { PathDefinitionType::Move ,   FOLLOW_RADIUS_SMALL              },

  //small half circles west
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },

  //turn around
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },

  //small half circles east
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_SMALL, -M_PI       },
  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_SMALL,  M_PI       },

  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M2_PI      },
  { PathDefinitionType::Arc  ,  -FOLLOW_RADIUS_LARGE, -M_PI_2     },

  { PathDefinitionType::Arc  ,   FOLLOW_RADIUS_LARGE,   M_PI      },
};
const int followPathLength = sizeof(followPath) / sizeof(PathDefinition);
