# Slew-Only Chrony Deployment

The checked-in `chrony.conf` is the physical-host policy. It contains no
`makestep` directive, so all corrections are slewed. Review the NTP pool for the
deployment network, then install and verify before starting ROS:

```bash
sudo apt update
sudo apt install chrony
getent ahostsv4 pool.ntp.org
sudo install -o root -g root -m 0644 deployment/chrony/chrony.conf /etc/chrony/chrony.conf
sudo systemctl disable --now systemd-timesyncd.service
sudo systemctl mask systemd-timesyncd.service
sudo systemctl enable --now chrony.service
sudo systemctl restart chrony.service
chronyc tracking
chronyc waitsync 60 0.05 5.0 1.0
```

The source lookup must return real NTP server addresses. Stop or reconfigure any
proxy fake-IP DNS mode if it returns an address in the reserved `198.18.0.0/15`
range. The policy caps polling at 64 seconds so clock-frequency confidence can
converge predictably on the physical host.

Inspect `/etc/chrony/chrony.conf` and every included file after installation;
qualification rejects any active `makestep` or `initstepslew` directive. Configure
the host's realtime limits and set every CPU governor to `performance` before
running the preflight. The checked-in realtime limits and installation steps are
under `deployment/realtime/`.

Do not run `chronyc makestep` while any robot process is active. Use
`start_hardware_stack.sh` for each physical stack command; it gates startup and
records the clock for that command. Direct ROS launches remain available for
fake hardware and simulation.
