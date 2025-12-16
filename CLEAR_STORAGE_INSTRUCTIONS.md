# How to Completely Clear Storage

## The Problem

Old profile data in localStorage is causing the level selection to be skipped even for new signups.

## Solution: Nuclear Clear

### Option 1: Use Browser DevTools

1. **Open DevTools** - Press `F12`

2. **Go to Application tab**

3. **Clear EVERYTHING:**
   - Click "Storage" in left sidebar
   - Click "Clear site data" button
   - Check ALL boxes:
     - ✅ Application cache
     - ✅ Cache storage
     - ✅ Cookies
     - ✅ Local storage
     - ✅ Session storage
   - Click "Clear site data"

4. **Verify it's cleared:**
   - Click "Local Storage" → http://localhost:3000
   - Should be EMPTY (no keys at all)

5. **Close DevTools**

6. **Hard refresh** - `Ctrl + Shift + R`

### Option 2: Incognito Window (Recommended)

1. **Close all incognito windows**
2. **Open NEW incognito window** (`Ctrl + Shift + N`)
3. **Go to** http://localhost:3000
4. **Sign up** with brand new email

This guarantees no cached data!

### Option 3: Manual localStorage Clear

1. **Open Console** - Press `F12` → Console tab

2. **Run this command:**
   ```javascript
   localStorage.clear()
   ```

3. **Verify:**
   ```javascript
   console.log(localStorage.length) // Should be 0
   ```

4. **Reload** - `Ctrl + Shift + R`

---

## After Clearing, Test This:

1. Go to http://localhost:3000
2. Click "Sign Up"
3. Fill form:
   ```
   Name: Test User
   Email: test999@example.com
   Password: password123
   ```
4. Click "Sign Up"
5. **Watch console** - Expand the "fullProfile" object
6. **Share what you see** in the fullProfile

The fullProfile should be either:
- `null` or `{}` (empty) for new user
- OR have `onboardingComplete: false`

If it has `onboardingComplete: true`, then something is setting it incorrectly.

---

## What the Console Should Show for NEW User:

```
Signup result: { data: {...}, error: null }
Showing level selection modal
Rendering LevelSelectionModal with name: Test User
LevelSelectionModal mounted, selectedLevel: null

AuthButton render - isAuthenticated: true userProfile: Object {
  hasProfile: false (or null)
  onboardingComplete: undefined (or false)
  difficultyLevel: undefined
}

User authenticated but onboarding not complete - keeping signup modal open
```

**If you see `onboardingComplete: true`**, that's the bug - we need to fix where it's being set!
