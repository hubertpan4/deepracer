# DeepRacer2024 development repo
[_Congrats Richard U., you've managed to nerd snipe me._](https://xkcd.com/356/)

This repo contains research notes, resources, and code related to the 2024 _redacted_ Atlanta DeepRacer competition.


## Getting Started
Research Notes: start with [deepRacerResearch.ipynb](deepRacerResearch.ipynb)

To get the python notebooks to execute I would setup a virtual environment 
and install the requirements with pip in the virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Personally I use [vsCode](https://code.visualstudio.com/) for all of my python development needs.


## Deployment
To "deploy" this project, upload the `reward_function.py` file to AWS