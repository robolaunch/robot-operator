package internal

import corev1 "k8s.io/api/core/v1"

// Platform meta labels
const (
	PLATFORM_VERSION_LABEL_KEY = "robolaunch.io/platform"
	DOMAIN_LABEL_KEY           = "robolaunch.io/domain"
)

// Tenancy labels
const (
	ORGANIZATION_LABEL_KEY         = "robolaunch.io/organization"
	TEAM_LABEL_KEY                 = "robolaunch.io/team"
	REGION_LABEL_KEY               = "robolaunch.io/region"
	CLOUD_INSTANCE_LABEL_KEY       = "robolaunch.io/cloud-instance"
	CLOUD_INSTANCE_ALIAS_LABEL_KEY = "robolaunch.io/cloud-instance-alias"
	PHYSICAL_INSTANCE_LABEL_KEY    = "robolaunch.io/physical-instance"
)

// Selector labels
const (
	ROS2_WORKLOAD_CONTAINER_SELECTOR_LABEL_KEY = "robolaunch.io/ros2-workload-container"
	CODE_EDITOR_CONTAINER_SELECTOR_LABEL_KEY   = "robolaunch.io/code-editor-container"
)

// Timezone labels
const (
	TZ_CONTINENT_LABEL_KEY = "robolaunch.io/tz-continent"
	TZ_CITY_LABEL_KEY      = "robolaunch.io/tz-city"
)

// Image labels
const (
	IMAGE_REGISTRY_LABEL_KEY   = "robolaunch.io/robot-image-registry"
	IMAGE_USER_LABEL_KEY       = "robolaunch.io/robot-image-user"
	IMAGE_REPOSITORY_LABEL_KEY = "robolaunch.io/robot-image-repository"
	IMAGE_TAG_LABEL_KEY        = "robolaunch.io/robot-image-tag"
)

// Target resource labels
const (
	TARGET_ROBOT_LABEL_KEY = "robolaunch.io/target-robot"
	TARGET_VDI_LABEL_KEY   = "robolaunch.io/target-vdi"
)

// Offline labels
const (
	OFFLINE_LABEL_KEY = "robolaunch.io/offline"
)

// Special escape labels
const (
	ROBOT_DEV_SUITE_OWNED = "robolaunch.io/dev-suite-owned"
)

// Platform server
const (
	DEFAULT_PLATFORM_URL           = "https://raw.githubusercontent.com/robolaunch/platform/main/platforms/industry-cloud-platform/platform.yaml"
	DEFAULT_ENVIRONMENTS_URL       = "https://raw.githubusercontent.com/robolaunch/platform/main/platforms/industry-cloud-platform/environments.yaml"
	DEFAULT_TOOLS_ENVIRONMENTS_URL = "https://raw.githubusercontent.com/robolaunch/platform/main/platforms/internal/environments.yaml"
)

// CodeEditor

const (
	CODE_EDITOR_APP_NAME  = "code-editor"
	CODE_EDITOR_PORT_NAME = "code-server"
)

// Robot owned resources' postfixes (v1alpha1)
const (
	PVC_VAR_POSTFIX           = "-pvc-var"
	PVC_ETC_POSTFIX           = "-pvc-etc"
	PVC_OPT_POSTFIX           = "-pvc-opt"
	PVC_USR_POSTFIX           = "-pvc-usr"
	PVC_DISPLAY_POSTFIX       = "-pvc-display"
	PVC_WORKSPACE_POSTFIX     = "-pvc-workspace"
	DISCOVERY_SERVER_POSTFIX  = "-discovery"
	JOB_LOADER_POSTFIX        = "-loader"
	ROS_BRIDGE_POSTFIX        = "-bridge"
	ROBOT_DEV_SUITE_POSTFIX   = "-dev"
	WORKSPACE_MANAGER_POSTFIX = "-ws"
)

// Robot owned resources' postfixes (v1alpha2)
const (
	ROS_2_BRIDGE_POSTFIX = "-bridge"
	PVC_POSTFIX          = "-pvc"
	STATEFULSET_POSTFIX  = ""
	DEPLOYMENT_POSTFIX   = ""
	SERVICE_POSTFIX      = ""
	INGRESS_POSTFIX      = ""
)

// WorkspaceManager owned resources' postfixes
const (
	JOB_CLONER_POSTFIX  = "-cloner"
	JOB_CLEANUP_POSTFIX = "-cleanup"
)

// BuildManager owned resources' postfixes
const (
	CONFIG_MAP_SCRIPTS = "-scripts"
)

// LaunchManager owned resources' postfixes
const (
	POD_LAUNCH_POSTFIX = "-launch"
)

// RobotVDI owned resources' postfixes
const (
	PVC_VDI_POSTFIX     = "-display"
	SVC_TCP_VDI_POSTFIX = "-tcp"
	SVC_UDP_VDI_POSTFIX = "-udp"
	POD_VDI_POSTFIX     = ""
	INGRESS_VDI_POSTFIX = ""
)

// RobotIDE owned resources' postfixes
const (
	SVC_IDE_POSTFIX                 = ""
	POD_IDE_POSTFIX                 = ""
	INGRESS_IDE_POSTFIX             = ""
	CUSTOM_PORT_SVC_IDE_POSTFIX     = "-custom"
	CUSTOM_PORT_INGRESS_IDE_POSTFIX = "-custom"
	CONFIGMAP_IDE_POSTFIX           = ""
)

// Notebook owned resources' postfixes
const (
	SVC_NOTEBOOK_POSTFIX                 = ""
	POD_NOTEBOOK_POSTFIX                 = ""
	INGRESS_NOTEBOOK_POSTFIX             = ""
	CUSTOM_PORT_SVC_NOTEBOOK_POSTFIX     = "-custom"
	CUSTOM_PORT_INGRESS_NOTEBOOK_POSTFIX = "-custom"
	CONFIGMAP_NOTEBOOK_POSTFIX           = ""
)

// RobotDevSuite owned resources' postfixes
const (
	ROBOT_VDI_POSTFIX               = "-vdi"
	ROBOT_IDE_POSTFIX               = "-ide"
	NOTEBOOK_POSTFIX                = "-notebook"
	REMOTE_IDE_RELAY_SERVER_POSTFIX = "-relay"
)

// Paths

const (
	CUSTOM_SCRIPTS_PATH = "/etc/custom"
	HELPERS_PATH        = "/var/lib/robolaunch-helpers/"
	X11_UNIX_PATH       = "/tmp/.X11-unix"
)

const (
	IMAGE_MAP_CONFIG_MAP_NAME      = "platform"
	IMAGE_MAP_CONFIG_MAP_NAMESPACE = "kube-system"
	IMAGE_MAP_CONFIG_MAP_DATA_KEY  = "platform.yaml"
	IMAGE_MAP_URL                  = "https://raw.githubusercontent.com/robolaunch/robolaunch/main/platform.yaml"
)

// Commands for collecting metrics
const (
	CMD_GET_CPU          = "cat /sys/fs/cgroup/cpu/cpuacct.usage"
	CMD_GET_MEMORY       = "cat /sys/fs/cgroup/memory/memory.usage_in_bytes"
	CMD_GET_NETWORK_LOAD = "cat /proc/net/dev | awk -F ' ' '{print $1 $2 \":\" $10}' | tail -n+3"
)

const (
	GRANT_PERMISSION_KEY           = "GRANT_PERMISSION"
	PERSISTENT_DIRS_KEY            = "PERSISTENT_DIRS"
	HOST_DIRS_KEY                  = "HOST_DIRS"
	IDE_CUSTOM_PORT_RANGE_KEY      = "IDE_CUSTOM_PORT_RANGE"
	VDI_CUSTOM_PORT_RANGE_KEY      = "VDI_CUSTOM_PORT_RANGE"
	NOTEBOOK_CUSTOM_PORT_RANGE_KEY = "NOTEBOOK_CUSTOM_PORT_RANGE"
)

// regex
const (
	GRANT_PERMISSION_REGEX  = "^(/([A-Za-z0-9./_-])+:)*(/[A-Za-z0-9./_-]+)$"
	PERSISTENT_DIRS_REGEX   = "^(/([A-Za-z0-9./_-])+:)*(/[A-Za-z0-9./_-]+)$"
	HOST_DIRS_REGEX         = "^(((/[A-Za-z0-9./_-]+):(/[A-Za-z0-9./_-]+))+,)*(((/[A-Za-z0-9./_-]+):(/[A-Za-z0-9./_-]+))+)$"
	CUSTOM_PORT_RANGE_REGEX = "^([a-z0-9]{4}-[0-9]{5}:[0-9]{2,5}/)*([a-z0-9]{4}-[0-9]{5}:[0-9]{2,5})$"
)

// file browser ports
const (
	FILE_BROWSER_PORT_NAME = "filebrowser"
	FILE_BROWSER_PORT      = 2000
)

// Ingress annotations
const (
	INGRESS_AUTH_URL_KEY                  = "nginx.ingress.kubernetes.io/auth-url"
	INGRESS_AUTH_URL_VAL                  = "https://%s.%s/oauth2/auth"
	INGRESS_AUTH_SIGNIN_KEY               = "nginx.ingress.kubernetes.io/auth-signin"
	INGRESS_AUTH_SIGNIN_VAL               = "https://%s.%s/oauth2/start?rd=$scheme://$best_http_host$request_uri"
	INGRESS_AUTH_RESPONSE_HEADERS_KEY     = "nginx.ingress.kubernetes.io/auth-response-headers"
	INGRESS_AUTH_RESPONSE_HEADERS_VAL     = "x-auth-request-user, x-auth-request-email, x-auth-request-access-token"
	INGRESS_CONFIGURATION_SNIPPET_KEY     = "nginx.ingress.kubernetes.io/configuration-snippet"
	INGRESS_PROXY_READ_TIMEOUT_KEY        = "nginx.ingress.kubernetes.io/proxy-read-timeout"
	INGRESS_PROXY_READ_TIMEOUT_VAL        = "7200"
	INGRESS_PROXY_SEND_TIMEOUT_KEY        = "nginx.ingress.kubernetes.io/proxy-send-timeout"
	INGRESS_PROXY_SEND_TIMEOUT_VAL        = "7200"
	INGRESS_VDI_CONFIGURATION_SNIPPET_VAL = "" +
		"        #proxy_set_header Host $host;\n" +
		"		proxy_set_header X-Real-IP $remote_addr;\n" +
		"		proxy_set_header X-Forwarded-For $remote_addr;\n" +
		"		proxy_set_header X-Forwarded-Host $host;\n" +
		"		proxy_set_header X-Forwarded-Port $server_port;\n" +
		"		proxy_set_header X-Forwarded-Protocol $scheme;\n"
	INGRESS_CERT_MANAGER_KEY                   = "acme.cert-manager.io/http01-edit-in-place"
	INGRESS_CERT_MANAGER_VAL                   = "true"
	INGRESS_NGINX_PROXY_BUFFER_SIZE_KEY        = "nginx.ingress.kubernetes.io/proxy-buffer-size"
	INGRESS_NGINX_PROXY_BUFFER_SIZE_VAL        = "16k"
	INGRESS_NGINX_PROXY_BUFFERS_NUMBER_KEY     = "nginx.ingress.kubernetes.io/proxy-buffers-number"
	INGRESS_VDI_NGINX_PROXY_BUFFERS_NUMBER_VAL = "4"
	INGRESS_NGINX_REWRITE_TARGET_KEY           = "nginx.ingress.kubernetes.io/rewrite-target"
	INGRESS_NGINX_REWRITE_TARGET_VAL           = "/$2"
	INGRESS_NGINX_PROXY_BODY_SIZE_KEY          = "nginx.ingress.kubernetes.io/proxy-body-size"
	INGRESS_NGINX_PROXY_BODY_SIZE_VAL          = "900m"

	INGRESS_IDE_CONFIGURATION_SNIPPET_VAL = "" +
		"auth_request_set $name_upstream_1 $upstream_cookie_name_1;" +
		"access_by_lua_block {" +
		"  if ngx.var.name_upstream_1 ~= \"\" then" +
		"	ngx.header[\"Set-Cookie\"] = \"name_1=\" .. ngx.var.name_upstream_1 .. ngx.var.auth_cookie:match(\"(; .*)\")" +
		"  end" +
		"}"
)

// File contents

// Super client configuration
const (
	SUPER_CLIENT_CONFIG = "" +
		"<?xml version='1.0' encoding='UTF-8' ?>" +
		"<dds>" +
		"	<profiles xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'>" +
		"		<transport_descriptors>" +
		"			<transport_descriptor>" +
		"				<transport_id>udp_transport</transport_id>" +
		"				<type>UDPv4</type>" +
		"			</transport_descriptor>" +
		"		</transport_descriptors>" +
		"		<participant profile_name='super_client_profile' is_default_profile='true'>" +
		"			<rtps>" +
		"				<userTransports>" +
		"					<transport_id>udp_transport</transport_id>" +
		"				</userTransports>" +
		"				<useBuiltinTransports>false</useBuiltinTransports>" +
		"				<builtin>" +
		"					<discovery_config>" +
		"						<discoveryProtocol>SUPER_CLIENT</discoveryProtocol>" +
		"						<discoveryServersList>" +
		"							<RemoteServer prefix='44.53.00.5f.45.50.52.4f.53.49.4d.41'>" +
		"								<metatrafficUnicastLocatorList>" +
		"									<locator>" +
		"										<udpv4>" +
		"										<address>" + "%s" + "</address>" +
		"											<port>11811</port>" +
		"										</udpv4>" +
		"									</locator>" +
		"								</metatrafficUnicastLocatorList>" +
		"							</RemoteServer>" +
		"						</discoveryServersList>" +
		"					</discovery_config>" +
		"				</builtin>" +
		"			</rtps>" +
		"		</participant>" +
		"	</profiles>" +
		"</dds>"
)

const (
	CUSTOM_SUPERVISORD_CONFIG = "" +
		"# replace your daemon's configuration" + "\n" +
		"# for reference, see http://supervisord.org/configuration.html" + "\n" +
		"[program:custom]" + "\n" +
		"environment=HOME='/home/robolaunch',USER='robolaunch',FILE_BROWSER_PORT='%(ENV_FILE_BROWSER_PORT)s'" + "\n" +
		"command=/bin/bash /etc/robolaunch/services/custom/custom.sh" + "\n" +
		"stopsignal=INT" + "\n" +
		"stopwaitsecs=5" + "\n" +
		"autorestart=true" + "\n" +
		"priority=800" + "\n" +
		"user=robolaunch" + "\n" +
		"stdout_logfile=/var/log/services/custom.log" + "\n" +
		"stdout_logfile_maxbytes=100MB" + "\n" +
		"stdout_logfile_backups=10" + "\n" +
		"redirect_stderr=true" + "\n"
	CUSTOM_BACKGROUND_SCRIPT = "" +
		"#!/bin/bash" + "\n" +
		"# replace your autostart script" + "\n" +
		"sleep infinity" + "\n"
)

func Bash(command string) []string {
	return []string{
		"/bin/bash",
		"-c",
		command,
	}
}

func Env(key string, value string) corev1.EnvVar {
	return corev1.EnvVar{
		Name:  key,
		Value: value,
	}
}
