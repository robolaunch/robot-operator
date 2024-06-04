package configure

import (
	"strconv"
	"strings"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
)

func (cfg *ContainerConfigInjector) InjectCustomPortConfiguration(container *corev1.Container, config robotv1alpha1.AdditionalConfig) *corev1.Container {

	// add custom ports defined by user
	portsSlice := strings.Split(config.Value, "/")
	for _, p := range portsSlice {
		portInfo := strings.Split(p, "-")
		portName := portInfo[0]
		fwdStr := strings.Split(portInfo[1], ":")
		// nodePortVal, _ := strconv.ParseInt(fwdStr[0], 10, 64)
		containerPortVal, _ := strconv.ParseInt(fwdStr[1], 10, 64)

		var protocol corev1.Protocol
		if strings.HasPrefix(portName, "t") {
			protocol = corev1.ProtocolTCP
		} else if strings.HasPrefix(portName, "u") {
			protocol = corev1.ProtocolUDP
		}

		container.Ports = append(container.Ports, corev1.ContainerPort{
			Name:          portName,
			ContainerPort: int32(containerPortVal),
			Protocol:      protocol,
		})
	}

	return container
}
