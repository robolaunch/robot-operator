package internal

// Tenancy labels
const (
	ORGANIZATION_LABEL_KEY      = "robolaunch.io/organization"
	TEAM_LABEL_KEY              = "robolaunch.io/team"
	REGION_LABEL_KEY            = "robolaunch.io/region"
	CLOUD_INSTANCE_LABEL_KEY    = "robolaunch.io/cloud-instance"
	PHYSICAL_INSTANCE_LABEL_KEY = "robolaunch.io/physical-instance"
)

// Ready robot properties
const (
	ROBOT_IMAGE = "robolaunch.io/robot-image"
)

// PVC postfixes
const (
	PVC_VAR_POSTFIX       = "-pvc-var"
	PVC_ETC_POSTFIX       = "-pvc-etc"
	PVC_OPT_POSTFIX       = "-pvc-opt"
	PVC_USR_POSTFIX       = "-pvc-usr"
	PVC_DISPLAY_POSTFIX   = "-pvc-display"
	PVC_WORKSPACE_POSTFIX = "-pvc-workspace"
)
