package configure

import (
	"strings"

	"github.com/robolaunch/robot-operator/internal"
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func GetGrantPermissionCmd(robot robotv1alpha1.Robot) string {

	var permissionCmdBuilder strings.Builder
	var closerStr string = ";"

	var paths []string
	if val, ok := robot.Spec.AdditionalConfigs[internal.GRANT_PERMISSION_KEY]; ok && val.ConfigType == robotv1alpha1.AdditionalConfigTypeOperator {
		paths = strings.Split(val.Value, ":")
	}

	for _, v := range paths {
		permissionCmdBuilder.WriteString("sudo setfacl -R -m u:robolaunch:rwx " + v + closerStr + " ")
	}

	permissionCmdBuilder.WriteString("sudo setfacl -R -m u:robolaunch:rwx " + "/etc/robolaunch" + closerStr + " ")

	return permissionCmdBuilder.String()
}
