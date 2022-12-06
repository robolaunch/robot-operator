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

// Robot owned resources' postfixes
const (
	PVC_VAR_POSTFIX          = "-pvc-var"
	PVC_ETC_POSTFIX          = "-pvc-etc"
	PVC_OPT_POSTFIX          = "-pvc-opt"
	PVC_USR_POSTFIX          = "-pvc-usr"
	PVC_DISPLAY_POSTFIX      = "-pvc-display"
	PVC_WORKSPACE_POSTFIX    = "-pvc-workspace"
	DISCOVERY_SERVER_POSTFIX = "-discovery"
)

// Super client configuration
const (
	SUPER_CLIENT_CONFIG = "<?xml version='1.0' encoding='UTF-8' ?>" +
		"<dds>" +
		"<profiles xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'>" +
		"<participant profile_name='super_client_profile' is_default_profile='true'>" +
		"<rtps>" +
		"		<builtin>" +
		"			<discovery_config>" +
		"				<discoveryProtocol>SUPER_CLIENT</discoveryProtocol>" +
		"				<discoveryServersList>" +
		"					<RemoteServer prefix='44.53.00.5f.45.50.52.4f.53.49.4d.41'>" +
		"						<metatrafficUnicastLocatorList>" +
		"							<locator>" +
		"								<udpv4>" +
		"								<address>" + "%s" + "</address>" +
		"									<port>11811</port>" +
		"								</udpv4>" +
		"							</locator>" +
		"						</metatrafficUnicastLocatorList>" +
		"					</RemoteServer>" +
		"				</discoveryServersList>" +
		"			</discovery_config>" +
		"		</builtin>" +
		"	</rtps>" +
		"</participant>" +
		"</profiles>" +
		"</dds>"
)
