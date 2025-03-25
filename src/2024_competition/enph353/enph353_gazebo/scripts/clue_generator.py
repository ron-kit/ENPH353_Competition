import requests
from openai import OpenAI
import csv
import os

def generateClues():
    '''
    '''

    '''
    URL = ""
    response = requests.get(URL)
    API_KEY,_ = response.text.split(',')
    '''

    # Using Cathy's key cus Miti's reached its quota
    script_dir = os.path.dirname(os.path.realpath(__file__))  # Get script directory
    api_key_path = os.path.join(script_dir, "api_key.txt")  # Construct absolute path

    with open(api_key_path, "r") as file:
        API_KEY = file.read().strip()

    client = OpenAI(
            api_key=API_KEY
        )

    prompt = f"""You will generate clues that describe a potential funny crime 
                for your game in random order. 
                The clues must have less than 13 characters and no more than 2 words. 
                Use themes from planet Earth.
                Display the clues in the following order:
                    NUMBER OF VICTIMS
                    WHO ARE THE VICTIMS
                    WHAT IS THE CRIME
                    WHEN WAS THE CRIME COMMITTED
                    WHERE WAS THE CRIME COMMITTED
                    WHY WAS THE CRIME COMMITED
                    WHAT WEAPON WAS THE CRIME COMMITED WITH
                    WHO WAS THE CRIMINAL

                    Return only the clues, separated by newlines.
                """

    completion = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": f"""You are a game creator making a 
             fun game similar to the boardgame Clue. You want the game to be
             different than the game of Clue so you make changes to it. Your
             game's victim has a funny name related to events from today. 
             This name can be common noun so don't use only proper nouns. 
             The criminals are also different than the original game of Clue 
             and they also make one smile. The location is not limited to 
             human scale, it can be anywhere in the universe from galaxies 
             to atomic nuclei. And the weapon needs to be a fun one too."""},
            {"role": "user", "content": prompt}
        ])

    story = completion.choices[0].message.content
    print(story)

    # Divide clues into csv format
    clues_list = story.split('\n')
    keys = ["SIZE", "VICTIM", "CRIME", "TIME", "PLACE", "MOTIVE", "WEAPON", "BANDIT"]
    clues = {key: clue.strip().upper() for key, clue in zip(keys, clues_list) if clue.strip()}

    # Write to clues.csv
    script_path = os.path.dirname(os.path.realpath(__file__)) + "/"
    with open(script_path + "clues.csv", 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for key, value in clues.items():
            writer.writerow([key, value])

generateClues()
exit(0)