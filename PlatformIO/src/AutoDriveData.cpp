
#include "AutoDriveData.h"

/* path for a single Zippy
double PATH_POINTS[PATH_POINT_COUNT][2] = {
  { 0.000000,0.000000 },
  { 0.080800,-0.200384 },
  { 0.133600,-0.314960 },
  { 0.186400,-0.396800 },
  { 0.239200,-0.445904 },
  { 0.240800,-0.446896 },
  { 0.311200,-0.474176 },
  { 0.416800,-0.490544 },
  { 0.557600,-0.496000 },
  { 0.562400,-0.496000 },
  { 0.703200,-0.485088 },
  { 0.808800,-0.452352 },
  { 0.879200,-0.397792 },
  { 0.880800,-0.395808 },
  { 0.924800,-0.308512 },
  { 0.951200,-0.177568 },
  { 0.960000,-0.002976 },
  { 0.960000,0.002976 },
  { 0.951200,0.177568 },
  { 0.924800,0.308512 },
  { 0.880800,0.395808 },
  { 0.879200,0.397792 },
  { 0.808800,0.452352 },
  { 0.703200,0.485088 },
  { 0.562400,0.496000 },
  { 0.557600,0.496000 },
  { 0.416800,0.490544 },
  { 0.311200,0.474176 },
  { 0.240800,0.446896 },
  { 0.239200,0.445904 },
  { 0.186400,0.396800 },
  { 0.133600,0.314960 },
  { 0.080800,0.200384 },
  { 0.079200,0.196416 },
  { 0.026400,0.065472 },
  { -0.026400,-0.065472 },
  { -0.079200,-0.196416 },
  { -0.080800,-0.200384 },
  { -0.133600,-0.314960 },
  { -0.186400,-0.396800 },
  { -0.239200,-0.445904 },
  { -0.240800,-0.446896 },
  { -0.311200,-0.474176 },
  { -0.416800,-0.490544 },
  { -0.557600,-0.496000 },
  { -0.562400,-0.496000 },
  { -0.703200,-0.485088 },
  { -0.808800,-0.452352 },
  { -0.879200,-0.397792 },
  { -0.880800,-0.395808 },
  { -0.924800,-0.308512 },
  { -0.951200,-0.177568 },
  { -0.960000,-0.002976 },
  { -0.960000,0.002976 },
  { -0.951200,0.177568 },
  { -0.924800,0.308512 },
  { -0.880800,0.395808 },
  { -0.879200,0.397792 },
  { -0.808800,0.452352 },
  { -0.703200,0.485088 },
  { -0.562400,0.496000 },
  { -0.557600,0.496000 },
  { -0.416800,0.490544 },
  { -0.311200,0.474176 },
  { -0.240800,0.446896 },
  { -0.239200,0.445904 },
  { -0.186400,0.396800 },
  { -0.133600,0.314960 },
  { -0.080800,0.200384 },
  { 0.000000,0.000000 },
};
*/

//dual-path for two side-by-side zippies
/*
double PATH_POINTS[PATH_POINT_COUNT][2] = {
  { 0.060000,0.000000 },
  { 0.135750,-0.171328 },
  { 0.184659,-0.269291 },
  { 0.232385,-0.339264 },
  { 0.278929,-0.381248 },
  { 0.280321,-0.382096 },
  { 0.341187,-0.405421 },
  { 0.431879,-0.419415 },
  { 0.552398,-0.424080 },
  { 0.556502,-0.424080 },
  { 0.676886,-0.414750 },
  { 0.767174,-0.386761 },
  { 0.827366,-0.340112 },
  { 0.828734,-0.338416 },
  { 0.866354,-0.263778 },
  { 0.888926,-0.151821 },
  { 0.896450,-0.002544 },
  { 0.896450,0.002544 },
  { 0.888926,0.151821 },
  { 0.866354,0.263778 },
  { 0.828734,0.338416 },
  { 0.827366,0.340112 },
  { 0.767174,0.386761 },
  { 0.676886,0.414750 },
  { 0.556502,0.424080 },
  { 0.552398,0.424080 },
  { 0.431879,0.419415 },
  { 0.341187,0.405421 },
  { 0.280321,0.382096 },
  { 0.278929,0.381248 },
  { 0.232385,0.339264 },
  { 0.184659,0.269291 },
  { 0.135750,0.171328 },
  { 0.134250,0.167936 },
  { 0.082550,0.052814 },
  { 0.026450,-0.068636 },
  { -0.034050,-0.196416 },
  { -0.035950,-0.200384 },
  { -0.097000,-0.314960 },
  { -0.154750,-0.396800 },
  { -0.209200,-0.445904 },
  { -0.210800,-0.446896 },
  { -0.282850,-0.474176 },
  { -0.393400,-0.490544 },
  { -0.542450,-0.496000 },
  { -0.547550,-0.496000 },
  { -0.696600,-0.485088 },
  { -0.807150,-0.452352 },
  { -0.879200,-0.397792 },
  { -0.880800,-0.395808 },
  { -0.924800,-0.308512 },
  { -0.951200,-0.177568 },
  { -0.960000,-0.002976 },
  { -0.960000,0.002976 },
  { -0.951200,0.177568 },
  { -0.924800,0.308512 },
  { -0.880800,0.395808 },
  { -0.879200,0.397792 },
  { -0.807150,0.452352 },
  { -0.696600,0.485088 },
  { -0.547550,0.496000 },
  { -0.542450,0.496000 },
  { -0.393400,0.490544 },
  { -0.282850,0.474176 },
  { -0.210800,0.446896 },
  { -0.209200,0.445904 },
  { -0.154750,0.396800 },
  { -0.097000,0.314960 },
  { -0.035950,0.200384 },
  { 0.060000,0.000000 },
};
*/

double PATH_POINTS[PATH_POINT_COUNT][2] = {
  { 0.060000,0.000000 },
  { 0.135750,-0.171328 },
  { 0.184659,-0.269291 },
  { 0.232385,-0.339264 },
  { 0.278929,-0.381248 },
  { 0.280321,-0.382096 },
  { 0.341187,-0.405421 },
  { 0.431879,-0.419415 },
  { 0.552398,-0.424080 },
  { 0.556502,-0.424080 },
  { 0.676886,-0.414750 },
  { 0.767174,-0.386761 },
  { 0.827366,-0.340112 },
  { 0.828734,-0.338416 },
  { 0.866354,-0.263778 },
  { 0.888926,-0.151821 },
  { 0.896450,-0.002544 },
  { 0.896450,0.002544 },
  { 0.888926,0.151821 },
  { 0.866354,0.263778 },
  { 0.828734,0.338416 },
  { 0.827366,0.340112 },
  { 0.767174,0.386761 },
  { 0.676886,0.414750 },
  { 0.556502,0.424080 },
  { 0.552398,0.424080 },
  { 0.431879,0.419415 },
  { 0.341187,0.405421 },
  { 0.280321,0.382096 },
  { 0.278929,0.381248 },
  { 0.232385,0.339264 },
  { 0.184659,0.269291 },
  { 0.135750,0.171328 },
  { 0.134250,0.167936 },
  { 0.082550,0.052814 },
  { 0.026450,-0.068636 },
  { -0.034050,-0.196416 },
  { -0.035950,-0.200384 },
  { -0.097000,-0.314960 },
  { -0.154750,-0.396800 },
  { -0.209200,-0.445904 },
  { -0.210800,-0.446896 },
  { -0.282850,-0.474176 },
  { -0.393400,-0.490544 },
  { -0.542450,-0.496000 },
  { -0.547550,-0.496000 },
  { -0.696600,-0.485088 },
  { -0.807150,-0.452352 },
  { -0.879200,-0.397792 },
  { -0.880800,-0.395808 },
  { -0.924800,-0.308512 },
  { -0.951200,-0.177568 },
  { -0.960000,-0.002976 },
  { -0.960000,0.002976 },
  { -0.951200,0.177568 },
  { -0.924800,0.308512 },
  { -0.880800,0.395808 },
  { -0.879200,0.397792 },
  { -0.807150,0.452352 },
  { -0.696600,0.485088 },
  { -0.547550,0.496000 },
  { -0.542450,0.496000 },
  { -0.393400,0.490544 },
  { -0.282850,0.474176 },
  { -0.210800,0.446896 },
  { -0.209200,0.445904 },
  { -0.154750,0.396800 },
  { -0.097000,0.314960 },
  { -0.035950,0.200384 },
  { -0.034050,0.196416 },
  { 0.018200,0.076296 },
  { 0.049550,-0.022176 },
  { 0.060000,-0.099000 },
  { 0.060000,-0.101000 },
  { 0.062750,-0.162875 },
  { 0.071000,-0.216500 },
  { 0.084750,-0.261875 },
  { 0.085250,-0.263125 },
  { 0.107250,-0.303000 },
  { 0.140250,-0.340125 },
  { 0.184250,-0.374500 },
  { 0.185750,-0.375500 },
  { 0.236625,-0.397500 },
  { 0.290250,-0.397500 },
  { 0.346625,-0.375500 },
  { 0.348375,-0.374500 },
  { 0.399250,-0.329125 },
  { 0.436375,-0.259000 },
  { 0.459750,-0.164125 },
  { 0.460250,-0.160875 },
  { 0.471250,-0.053625 },
  { 0.471250,0.053625 },
  { 0.460250,0.160875 },
  { 0.459750,0.164125 },
  { 0.436375,0.259000 },
  { 0.399250,0.329125 },
  { 0.348375,0.374500 },
  { 0.346625,0.375500 },
  { 0.290250,0.397500 },
  { 0.236625,0.397500 },
  { 0.185750,0.375500 },
  { 0.184250,0.374500 },
  { 0.140250,0.340125 },
  { 0.107250,0.303000 },
  { 0.085250,0.263125 },
  { 0.084750,0.261875 },
  { 0.071000,0.216500 },
  { 0.062750,0.162875 },
  { 0.060000,0.101000 },
  { 0.060000,0.099000 },
  { 0.060000,0.033000 },
  { 0.060000,-0.033000 },
  { 0.060000,-0.099000 },
  { 0.060000,-0.101000 },
  { 0.057250,-0.162875 },
  { 0.049000,-0.216500 },
  { 0.035250,-0.261875 },
  { 0.034750,-0.263125 },
  { 0.012750,-0.303000 },
  { -0.020250,-0.340125 },
  { -0.064250,-0.374500 },
  { -0.065750,-0.375500 },
  { -0.116625,-0.397500 },
  { -0.170250,-0.397500 },
  { -0.226625,-0.375500 },
  { -0.228375,-0.374500 },
  { -0.279250,-0.329125 },
  { -0.316375,-0.259000 },
  { -0.339750,-0.164125 },
  { -0.340250,-0.160875 },
  { -0.351250,-0.053625 },
  { -0.351250,0.053625 },
  { -0.340250,0.160875 },
  { -0.339750,0.164125 },
  { -0.316375,0.259000 },
  { -0.279250,0.329125 },
  { -0.228375,0.374500 },
  { -0.226625,0.375500 },
  { -0.170250,0.397500 },
  { -0.116625,0.397500 },
  { -0.065750,0.375500 },
  { -0.064250,0.374500 },
  { -0.020250,0.340125 },
  { 0.012750,0.303000 },
  { 0.034750,0.263125 },
  { 0.035250,0.261875 },
  { 0.049000,0.216500 },
  { 0.057250,0.162875 },
  { 0.060000,0.101000 },
  { 0.060000,0.099000 },
  { 0.059340,0.038500 },
  { 0.057360,-0.011000 },
  { 0.054060,-0.049500 },
  { 0.053940,-0.050500 },
  { 0.048440,-0.081438 },
  { 0.039860,-0.108250 },
  { 0.028200,-0.130938 },
  { 0.027800,-0.131563 },
  { 0.010200,-0.151500 },
  { -0.016200,-0.170063 },
  { -0.051400,-0.187250 },
  { -0.052600,-0.187750 },
  { -0.093300,-0.198750 },
  { -0.136200,-0.198750 },
  { -0.181300,-0.187750 },
  { -0.182700,-0.187250 },
  { -0.223400,-0.164563 },
  { -0.253100,-0.129500 },
  { -0.271800,-0.082063 },
  { -0.272200,-0.080438 },
  { -0.281000,-0.026813 },
  { -0.281000,0.026812 },
  { -0.272200,0.080437 },
  { -0.271800,0.082062 },
  { -0.253100,0.129500 },
  { -0.223400,0.164563 },
  { -0.182700,0.187250 },
  { -0.181300,0.187750 },
  { -0.136200,0.198750 },
  { -0.093300,0.198750 },
  { -0.052600,0.187750 },
  { -0.051400,0.187250 },
  { -0.016200,0.170063 },
  { 0.010200,0.151500 },
  { 0.027800,0.131563 },
  { 0.028200,0.130937 },
  { 0.039860,0.108250 },
  { 0.048440,0.081437 },
  { 0.053940,0.050500 },
  { 0.054060,0.049500 },
  { 0.056502,0.017600 },
  { 0.055908,-0.012100 },
  { 0.052278,-0.039600 },
  { 0.052122,-0.040400 },
  { 0.046072,-0.065150 },
  { 0.038218,-0.086600 },
  { 0.028560,-0.104750 },
  { 0.028240,-0.105250 },
  { 0.014160,-0.121200 },
  { -0.006960,-0.136050 },
  { -0.035120,-0.149800 },
  { -0.036080,-0.150200 },
  { -0.068640,-0.159000 },
  { -0.102960,-0.159000 },
  { -0.139040,-0.150200 },
  { -0.140160,-0.149800 },
  { -0.172720,-0.131650 },
  { -0.196480,-0.103600 },
  { -0.211440,-0.065650 },
  { -0.211760,-0.064350 },
  { -0.218800,-0.021450 },
  { -0.218800,0.021450 },
  { -0.211760,0.064350 },
  { -0.211440,0.065650 },
  { -0.196480,0.103600 },
  { -0.172720,0.131650 },
  { -0.140160,0.149800 },
  { -0.139040,0.150200 },
  { -0.102960,0.159000 },
  { -0.068640,0.159000 },
  { -0.036080,0.150200 },
  { -0.035120,0.149800 },
  { -0.006960,0.136050 },
  { 0.014160,0.121200 },
  { 0.028240,0.105250 },
  { 0.028560,0.104750 },
  { 0.037888,0.086600 },
  { 0.044752,0.065150 },
  { 0.049152,0.040400 },
  { 0.054000,0.000000 },
};
