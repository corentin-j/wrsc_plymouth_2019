# Wind

## Direction

Used an extended kalman filter

## Speed

Sometimes there are some randome data such as 123, 216, etc ... when the wind speed is around 5 or 6 meters/seceonds. A reactivity was needed, that is why a threshold has been applyied rather than a low_pass filter

Return the true wind and not the apparent wind. It need the speed given by the GPS to compute the true wind. If there is no value, the apparent wind is returned.