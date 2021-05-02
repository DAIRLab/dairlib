from __future__ import print_function
import sys
from datetime import datetime
import string
import os
import subprocess

import pickle
import os.path
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from httplib2 import Http
from apiclient.http import MediaFileUpload

# ONLY COMPATIBLE WITH PYTHON3

# Determines the range of authorization for this program. Currently, this file can manipulate drive files and edit google sheets.
SCOPES = 'https://www.googleapis.com/auth/spreadsheets https://www.googleapis.com/auth/drive.file'

# Log spreadsheet ID
SPREADSHEET_ID = '1oZMdnEAaeDzIHSmzrWswwlkcAY0fknxTYA6QMBn1-JM'
# Search range used to find current test number (Basically all numbers in first column)
RANGE_NAME = 'Sheet1!A1:A1000'

def main():
    # fetches most recent remote repo
    os.system("git fetch")

    # Checks status of local branch in relation to remote branch by calling 'git status' in the terminal
    git_updated = True
    out = subprocess.getoutput(['git status', 'l'])

    # Checks for various keywords that would indicate the local repo is not up to date
    if "untracked files present" in out:
        print ("Untracked Files Present.")
        git_updated = False
    if "Changes to be committed" in out:
        print("Uncommitted changes in local workspace.")
        git_updated = False
    if "Changes not staged for commit" in out:
        print("Unstaged file changes in workspace")
        git_updated = False
    if "ahead of " in out:
        print("Your branch is ahead of remote repository.")
        git_updated = False

    # If the local repo is not up to date, gives the user the option to continue anyway.
    if git_updated:
        print("Branch is up to date!")
    else:
        decision = input("Enter 'yes' to continue: ")
        if decision != "yes":
            exit()

    # Finds branch name
    firstLine = "On branch "
    branch = out[(out.find(firstLine) + len(firstLine)):out.find("\n")]

    """
    # Finds location of current branch ie. "origin" or "upstream"
    cut = out[out.find("'"):out.find(".")]
    stripped = cut.strip("'")
    origin = stripped[:stripped.find("/")]

    # Returns url of remote repo
    out1 = subprocess.getoutput(['git config --get remote.' + origin + '.url', 'l'])
    repo = out1.replace(".git", "").replace("https://github.com/", "")
    print(branch + " " + repo)
    """

    # Return first six digits of hash
    sha = subprocess.getoutput(['git rev-parse HEAD', 'l'])[:6]

    creds = None
    # The file token.pickle stores the user's access and refresh tokens, and is
    # created automatically when the authorization flow completes for the first
    # time.
    if os.path.exists('logtoken1.pickle'):
        with open('logtoken1.pickle', 'rb') as token:
            creds = pickle.load(token)
    # If there are no (valid) credentials available, let the user log in.
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'config/realupwrite.json', SCOPES)
            creds = flow.run_local_server()
        # Saves the credentials to logtoken1.pickle
        with open('logtoken1.pickle', 'wb') as token:
            pickle.dump(creds, token)

    # Builds google sheets and drive APIs.
    service = build('sheets', 'v4', credentials=creds)
    DRIVE = build('drive', 'v3', credentials=creds)

    # Returns length of first column of spreadsheet, thereby providing the current test number assuming
    # the log sheet is up to date.
    col1 = service.spreadsheets().values().get(spreadsheetId=SPREADSHEET_ID, range=RANGE_NAME).execute()
    column = col1.get('values',[])

    # Runs lcm-logger, creates log final upon completion.
    print("Press ctrl + c to exit lcm-logger when experiment is complete.")
    time = str(datetime.now()).replace(" ", "")
    os.system('/opt/lcm/1.3.95.20180523/bin/lcm-logger test' + str(len(column)) + '@' + time + '.log')

    # Uploads lcm log file to Google Drive.
    lcmfileName = 'test' + str(len(column)) + '@' + time + '.log'
    folder_id = '10DH0fMYXZZ03kRClNkVQhP8KMUBzJPtF'
    metadata = {
        'name': lcmfileName,
        'parents': [folder_id]
    }
    media = MediaFileUpload(lcmfileName, mimetype='application/octet-stream')
    res = DRIVE.files().create(body=metadata, media_body=media, fields='id').execute()

    # Initalizes formatted date and time strings
    date = datetime.now().strftime('%y-%m-%d ')
    time = datetime.now().strftime('%I:%M %p')

    # Uploads kuka settings file to Google Drive.
    config_file = 'simulation_settings.json'
    folder_id1 = '10O80Ue9wcQOEu4EKbQW7IG4Rnj55oaRm'
    config_drive_file_name = config_file + date + time
    metadata = {
        'name' : config_drive_file_name,
        'parents': [folder_id1]
    }
    media = MediaFileUpload(config_file, mimetype='text/plain')
    res = DRIVE.files().create(body=metadata, media_body=media, fields='id').execute()

    # Uploads trajectories file to Google Drive.
    trajectories_file = 'Trajectories.csv'
    folder_id2 = '1uvtnitohhBak9PavuX2jhznHjuFSYAR7'
    trajectories_drive_name = trajectories_file + "_" + date + "_" + time
    metadata = {
        'name': trajectories_drive_name,
        'parents': [folder_id2]
    }
    media = MediaFileUpload(trajectories_file, mimetype='text/plain')
    res = DRIVE.files().create(body=metadata, media_body=media, fields='id').execute()

    # Creates google sheets print range for the next test log (the next unedited row)
    printRange = 'Sheet1!A' + str(len(column) + 1) + ':K' + str(len(column) + 1)

    # Initializes test number variable
    testNum = str(len(column))

    # Prompts the user for missing data.
    print("Test #" + str(len(column)) + ": ")
    description = input("Description: ")
    simulated = input("Simulated Before?: ")
    result = input("Result: ")
    # git_repository = input("Git Repository: ")
    giturl = subprocess.getoutput(['git config --get remote.origin.url'])
    giturl = giturl[(giturl.find(':')+1):]
    giturl = giturl[:giturl.find('.')]
    git_repository = giturl
    # Initializes body of the update (organizes each value into columns)
    body = {
    "values": [
        [testNum, date, time, description, simulated, lcmfileName, config_drive_file_name, result, git_repository, branch, sha]
    ],
    "majorDimension": "ROWS",
    "range": printRange
    }

    # Updates the log spreadsheet.
    request = service.spreadsheets().values().update(spreadsheetId=SPREADSHEET_ID, range=printRange,
        valueInputOption='RAW', body=body)
    response = request.execute()

if __name__== "__main__": main()
