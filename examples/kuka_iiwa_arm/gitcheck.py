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

# If modifying these scopes, delete the file token.pickle.
SCOPES = 'https://www.googleapis.com/auth/spreadsheets https://www.googleapis.com/auth/drive.file'

# Log spreadsheet ID
SPREADSHEET_ID = '1oZMdnEAaeDzIHSmzrWswwlkcAY0fknxTYA6QMBn1-JM'
# Search range used to find current test number (All numbers in first column)
RANGE_NAME = 'Sheet1!A1:A100'

def main():
    #Checks status of local branch in relation to remote branch.
    #os.system("git fetch")
    #print("git fetch")
    out = subprocess.getoutput(['git status', 'l'])

    if "untracked files present" in out:
        print ("Untracked Files Present. Please add and commit file changes.")
        exit()
    if "Changes to be committed" in out:
        print("Uncommitted changes in local workspace.")
        exit()
    if "Changes not staged for commit" in out:
        print("Unstaged file changes in workspace")
        exit()
    if "ahead of " in out:
        print("Your branch is ahead of remote repository.")
        exit()
    else:
        print("Branch is up to date!")
    
    # Finds branch name
    firstLine = "On branch "
    branch = out[(out.find(firstLine) + len(firstLine)):out.find("\n")]
    
    # Finds location of current branch ie. "origin" or "upstream"
    cut = out[out.find("'"):out.find(".")]
    stripped = cut.strip("'")
    origin = stripped[:stripped.find("/")]
    
    # Returns url of remote repo
    out1 = subprocess.getoutput(['git config --get remote.' + origin + '.url', 'l'])
    repo = out1.replace(".git", "").replace("https://github.com/", "")
    print(branch + " " + repo)

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
                'realupwrite.json', SCOPES)
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
    os.system('lcm-logger test' + str(len(column)) + '@' + time + '.log')

    # Uploads lcm log file to Google Drive.
    fileName = 'test' + str(len(column)) + '@' + time + '.log'
    metadata = {'name': fileName}
    media = MediaFileUpload(fileName, mimetype='application/octet-stream')
    res = DRIVE.files().create(body=metadata, media_body=media, fields='id').execute()

    # Creates the print range for the next test log (the next unedited row)
    printRange = 'Sheet1!A' + str(len(column) + 1) + ':K' + str(len(column) + 1)
    
    # Automatically creates test number, date, and time
    testNum = str(len(column))
    date = datetime.now().strftime('%m/%d/%y')
    time = datetime.now().strftime('%I:%M %p')

    # Prompts the user for missing data.
    print("Test #" + str(len(column)) + ": ")
    description = input("Description: ")
    simulated = input("Simulated Before?: ")
    lcmfile = 'test' + str(len(column)) + '.log'
    configfile = input("Test Config Log File: ")
    result = input("Result: ")

    # Initializes body of the update (organizes each value into columns)
    body = {
    "values": [
        [testNum, date, time, description, simulated, lcmfile, configfile, result, repo, branch, sha]
    ],
    "majorDimension": "ROWS",
    "range": printRange
    }

    # Updates the log spreadsheet.
    request = service.spreadsheets().values().update(spreadsheetId=SPREADSHEET_ID, range=printRange,
        valueInputOption='RAW', body=body)
    response = request.execute()

if __name__== "__main__": main()