# deepracer

* <https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-build-your-track.html>
* <https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-vehicle-factory-reset-preparation.html>

* <https://github.com/aws-samples/aws-deepracer-workshops>

* <https://github.com/aws-deepracer-community/deepracer-simapp>

* <https://github.com/cdthompson/deepracer-k1999-race-lines>
* <https://github.com/matrousseau/AWS-Deepracer-Optimal-Path-Generator>
* <https://github.com/dgnzlz/Capstone_AWS_DeepRacer>

## python

```bash
pyenv install 3.10.10
pyenv global 3.10.10

python --version
python -m pip install --upgrade pip

python -m pip install -r requirements.txt
```

## insight

```sql
fields episode, steps, x, y, total, time, diff_progress, name
| filter progress < 0 # and name =~ 'dr-'
| order by diff_progress desc, time

fields episode, steps, x, y, total, progress, time, reward, speed, steering_angle, diff_progress
| filter progress > 0 # and name =~ 'dr-'
| order by episode desc, steps desc

fields episode, steps, x, y, total, progress, time, reward, speed, steering_angle, diff_progress
| filter progress > 0 and name == 'dr-0418' and episode == 65
| order by steps

fields @timestamp, @message
| filter @message =~ 'SIM_TRACE_LOG' and @message =~ '0,True'
| order by @timestamp desc, @message desc
```

## log download

```bash
aws logs get-log-events \
    --log-group-name "/aws/robomaker/SimulationJobs" \
    --log-stream-name "<STREAM_NAME>" \
    --output text \
    --region us-east-1 > deepracer-sim.log

aws logs filter-log-events \
    --log-group-name "/aws/robomaker/SimulationJobs" \
    --log-stream-name "<STREAM_NAME>" \
    --filter-pattern SIM_TRACE_LOG \
    --output text \
    --region us-east-1 > deepracer-sim.log
```

## track

```bash
cat reinvent.json | jq -r '.waypoints[] | "\(.[0]),\(.[1])"'
```

## ssh

```bash
sudo ufw allow 22/tcp
```

## npy

```python
import numpy as np

x = np.load('Aragon.npy')
np.savetxt('Aragon.csv', x, delimiter=',')

y = np.loadtxt('track.csv', delimiter=',')
np.save('track.npy', y)
```

## s3

```bash
aws s3 sync ./model s3://aws-deepracer-nalbam/model/

aws s3 sync "s3://aws-deepracer-nalbam/model/Tue, 22 Aug 2023 13:04:10 GMT/" ./logs/model/
```

## ipykernel

```bash
python -m pip install ipykernel --force-reinstall
```
