for n in 0 1 2 3; do
  echo "=== /dev/video$n ==="
  v4l2-ctl -d /dev/video$n --list-formats-ext | head -20
done
