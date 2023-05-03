# Discovery Server Configuration

Here is the FastDDS configuration that provides communications such that

- clients will only consume UDPv4
- as super client
- using discovery server

```xml
<?xml version='1.0' encoding='UTF-8' ?>
<dds>
    <profiles
        xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'>
        
        <!-- UDPv4 Transport profile -->
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        
        <participant profile_name='super_client_profile' is_default_profile='true'>
            <rtps>
                <!-- Use user defined UDPv4 transport -->
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <!-- Disable builtin transports -->
                <useBuiltinTransports>false</useBuiltinTransports>

                <!-- Connect to discovery server as SUPER_CLIENT -->
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix='44.53.00.5f.45.50.52.4f.53.49.4d.41'>
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>POD_IP</address>
                                            <port>CONTAINER_PORT</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

This solution is recommended in [FastDDS discussions](https://github.com/eProsima/Fast-DDS/discussions/3262).